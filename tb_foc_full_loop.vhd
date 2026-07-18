--------------------------------------------------------------------------------
-- tb_foc_full_loop.vhd
--
-- Capstone integration test: wires all six modules into the complete
-- signal chain and checks a full control step end-to-end, from raw
-- phase currents and rotor angle all the way to PWM duty cycles:
--
--   (ia, ib) --Clarke--> (Ialpha,Ibeta) --Park(+CORDIC cos/sin)--> (Id,Iq)
--   (Id_ref-Id, Iq_ref-Iq) --PI(d), PI(q)--> (Vd,Vq)
--   (Vd,Vq) --InversePark(+ same cos/sin)--> (Valpha,Vbeta) --SVPWM--> duty a/b/c
--
-- Each module's own math was already verified in isolation (and in
-- smaller sub-chains); this test is about proving the *wiring* -- that
-- composing all six blocks together produces the same result as
-- computing the whole thing by hand in floating point, for a first
-- control step from a freshly reset PI state. A full closed-loop
-- dynamic run needs a motor plant model, which is separate future work
-- (see Part 1/README).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_foc_full_loop is
end entity tb_foc_full_loop;

architecture sim of tb_foc_full_loop is

    constant DW  : integer := 16;
    constant AW  : integer := 22;
    constant GW  : integer := 16;
    constant GF  : integer := 11;
    constant CLK_PERIOD : time := 10 ns;
    constant Q15 : real := 32768.0;
    constant Q18 : real := 262144.0;
    constant Q11 : real := 2048.0;
    constant TOL  : integer := 10;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal cl_vin : std_logic := '0';
    signal ia, ib : signed(DW-1 downto 0) := (others => '0');
    signal cl_vout : std_logic;
    signal ialpha, ibeta : signed(DW-1 downto 0);

    signal cd_vin : std_logic := '0';
    signal theta_q18 : signed(AW-1 downto 0) := (others => '0');
    signal cd_busy, cd_vout : std_logic;
    signal cos_th, sin_th : signed(DW-1 downto 0);

    signal pk_vin : std_logic := '0';
    signal pk_vout : std_logic;
    signal id_meas, iq_meas : signed(DW-1 downto 0);

    signal pid_vin, piq_vin : std_logic := '0';
    signal err_d, err_q : signed(DW-1 downto 0) := (others => '0');
    signal kp_d, ki_d, kp_q, ki_q : signed(GW-1 downto 0) := (others => '0');
    signal pid_vout, piq_vout : std_logic;
    signal vd, vq : signed(DW-1 downto 0);

    signal ipk_vin : std_logic := '0';
    signal ipk_vout : std_logic;
    signal valpha, vbeta : signed(DW-1 downto 0);

    signal sv_vin : std_logic := '0';
    signal sv_vout : std_logic;
    signal duty_a, duty_b, duty_c : unsigned(DW-1 downto 0);

    signal sim_done : boolean := false;

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x*Q15));
        if v > 32767 then v:=32767; end if;
        if v < -32768 then v:=-32768; end if;
        return to_signed(v, DW);
    end function;
    function to_q18 (x : real) return signed is
    begin
        return to_signed(integer(round(x*Q18)), AW);
    end function;
    function to_q11 (x : real) return signed is
    begin
        return to_signed(integer(round(x*Q11)), GW);
    end function;

begin

    u_clarke : entity work.clarke_transform
        generic map (DATA_WIDTH=>DW)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>cl_vin, i_a=>ia, i_b=>ib,
                  o_valid=>cl_vout, o_alpha=>ialpha, o_beta=>ibeta);

    u_cordic : entity work.cordic_sincos
        port map (clk=>clk, rst_n=>rst_n, i_valid=>cd_vin, i_theta=>theta_q18,
                  o_busy=>cd_busy, o_valid=>cd_vout, o_cos=>cos_th, o_sin=>sin_th);

    u_park : entity work.park_transform
        generic map (DATA_WIDTH=>DW)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>pk_vin,
                  i_alpha=>ialpha, i_beta=>ibeta, i_cos=>cos_th, i_sin=>sin_th,
                  o_valid=>pk_vout, o_d=>id_meas, o_q=>iq_meas);

    u_pi_d : entity work.pi_controller
        generic map (DATA_WIDTH=>DW, GAIN_WIDTH=>GW, GAIN_FRAC=>GF)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>pid_vin, i_error=>err_d,
                  i_kp=>kp_d, i_ki=>ki_d, o_valid=>pid_vout, o_output=>vd);

    u_pi_q : entity work.pi_controller
        generic map (DATA_WIDTH=>DW, GAIN_WIDTH=>GW, GAIN_FRAC=>GF)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>piq_vin, i_error=>err_q,
                  i_kp=>kp_q, i_ki=>ki_q, o_valid=>piq_vout, o_output=>vq);

    u_inv_park : entity work.inverse_park_transform
        generic map (DATA_WIDTH=>DW)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>ipk_vin,
                  i_d=>vd, i_q=>vq, i_cos=>cos_th, i_sin=>sin_th,
                  o_valid=>ipk_vout, o_alpha=>valpha, o_beta=>vbeta);

    u_svpwm : entity work.svpwm
        generic map (DATA_WIDTH=>DW)
        port map (clk=>clk, rst_n=>rst_n, i_valid=>sv_vin,
                  i_valpha=>valpha, i_vbeta=>vbeta,
                  o_valid=>sv_vout, o_duty_a=>duty_a, o_duty_b=>duty_b, o_duty_c=>duty_c);

    clk_gen : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD/2;
            clk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    stim : process
        variable theta_rad : real;
        variable ia_r, ib_r : real := 0.0;
        variable id_ref, iq_ref : real;
        variable c, s : real;
        variable ialpha_r, ibeta_r, id_r, iq_r : real;
        variable errd_r, errq_r, vd_r, vq_r : real;
        variable valpha_r, vbeta_r : real;
        variable va_eff, vb_eff, vc_eff, achk, bchk : real;
        variable kp_r, ki_r : real := 0.0;
        variable pass_count, fail_count : integer := 0;

        procedure check (lbl : string; got_r, exp_r : real; tol_lsb : integer) is
            variable got_q, exp_q : integer;
        begin
            got_q := to_integer(to_q15(got_r));
            exp_q := to_integer(to_q15(exp_r));
            if abs(got_q - exp_q) <= tol_lsb then
                pass_count := pass_count + 1;
                report "PASS " & lbl & "  got=" & integer'image(got_q) & " exp=" & integer'image(exp_q);
            else
                fail_count := fail_count + 1;
                report "FAIL " & lbl & "  got=" & integer'image(got_q) & " exp=" & integer'image(exp_q)
                    severity error;
            end if;
        end procedure;

    begin
        rst_n <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        -- scenario: raw currents/angle -> measured Id,Iq; PI gains and
        -- current references chosen so nothing saturates, to keep the
        -- expected-value arithmetic simple to cross-check by hand.
        ia_r := 0.4; ib_r := -0.2; theta_rad := 35.0 * MATH_PI/180.0;
        id_ref := 0.5; iq_ref := 0.1;
        kp_r := 0.8; ki_r := 0.05;

        c := cos(theta_rad); s := sin(theta_rad);

        -- 1) Clarke + CORDIC (parallel)
        wait until rising_edge(clk);
        ia <= to_q15(ia_r); ib <= to_q15(ib_r); cl_vin <= '1';
        theta_q18 <= to_q18(theta_rad); cd_vin <= '1';
        wait until rising_edge(clk);
        cl_vin <= '0'; cd_vin <= '0';
        wait until cl_vout = '1';
        wait until cd_vout = '1';

        -- 2) Park
        wait until rising_edge(clk);
        pk_vin <= '1';
        wait until rising_edge(clk);
        pk_vin <= '0';
        wait until pk_vout = '1';

        ialpha_r := ia_r;
        ibeta_r  := (ia_r + 2.0*ib_r)/sqrt(3.0);
        id_r := ialpha_r*c + ibeta_r*s;
        iq_r := -ialpha_r*s + ibeta_r*c;
        check("Id (measured)", real(to_integer(id_meas))/Q15, id_r, TOL);
        check("Iq (measured)", real(to_integer(iq_meas))/Q15, iq_r, TOL);

        -- 3) PI controllers (first step from reset: e_prev=0, y_prev=0)
        errd_r := id_ref - id_r;
        errq_r := iq_ref - iq_r;
        kp_d <= to_q11(kp_r); ki_d <= to_q11(ki_r);
        kp_q <= to_q11(kp_r); ki_q <= to_q11(ki_r);

        wait until rising_edge(clk);
        err_d <= to_q15(errd_r); pid_vin <= '1';
        err_q <= to_q15(errq_r); piq_vin <= '1';
        wait until rising_edge(clk);
        pid_vin <= '0'; piq_vin <= '0';
        wait until (pid_vout = '1') and (piq_vout = '1');

        vd_r := kp_r*errd_r + ki_r*errd_r;   -- 1st-step incremental PI, e_prev=0
        vq_r := kp_r*errq_r + ki_r*errq_r;
        check("Vd (PI output)", real(to_integer(vd))/Q15, vd_r, TOL);
        check("Vq (PI output)", real(to_integer(vq))/Q15, vq_r, TOL);

        -- 4) Inverse Park
        wait until rising_edge(clk);
        ipk_vin <= '1';
        wait until rising_edge(clk);
        ipk_vin <= '0';
        wait until ipk_vout = '1';

        valpha_r := vd_r*c - vq_r*s;
        vbeta_r  := vd_r*s + vq_r*c;
        check("Valpha", real(to_integer(valpha))/Q15, valpha_r, TOL);
        check("Vbeta",  real(to_integer(vbeta))/Q15,  vbeta_r,  TOL);

        -- 5) SVPWM
        wait until rising_edge(clk);
        sv_vin <= '1';
        wait until rising_edge(clk);
        sv_vin <= '0';
        wait until sv_vout = '1';

        va_eff := 2.0*(real(to_integer(duty_a))/65536.0) - 1.0;
        vb_eff := 2.0*(real(to_integer(duty_b))/65536.0) - 1.0;
        vc_eff := 2.0*(real(to_integer(duty_c))/65536.0) - 1.0;
        achk := (2.0*va_eff - vb_eff - vc_eff)/3.0;
        bchk := (vb_eff - vc_eff)/sqrt(3.0);
        check("SVPWM->Valpha reconstruction", achk, valpha_r, TOL);
        check("SVPWM->Vbeta reconstruction",  bchk, vbeta_r,  TOL);

        report "================================================";
        report "FULL FOC LOOP (Clarke->CORDIC->Park->PI->InvPark->SVPWM): " &
               integer'image(pass_count) & " passed, " & integer'image(fail_count) & " failed";
        report "================================================";

        assert fail_count = 0
            report "FULL LOOP INTEGRATION TEST FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
