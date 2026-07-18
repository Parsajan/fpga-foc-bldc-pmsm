--------------------------------------------------------------------------------
-- tb_svpwm.vhd
--
-- Self-checking testbench for svpwm.vhd. Rather than reproducing a
-- sector-table by hand (easy to get subtly wrong, and then you're just
-- checking one derivation against another with the same risk of error
-- in both), this checks the module against its own defining property:
-- the average voltage the duty cycles would synthesize, reconstructed
-- through the (zero-sequence-rejecting) full Clarke transform, must
-- equal the original commanded Valpha/Vbeta. This was cross-checked
-- offline in Python against a conventional sector-based implementation
-- to 1e-16 agreement before committing to this approach.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_svpwm is
end entity tb_svpwm;

architecture sim of tb_svpwm is

    constant DATA_WIDTH : integer := 16;
    constant CLK_PERIOD : time    := 10 ns;
    constant Q15_SCALE  : real    := 32768.0;
    constant TOL_LSB    : integer := 4;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal i_valid : std_logic := '0';
    signal i_valpha, i_vbeta : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal o_valid : std_logic;
    signal o_duty_a, o_duty_b, o_duty_c : unsigned(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    type real_array is array (natural range <>) of real;
    -- angles across all 6 sectors, magnitudes from small to the max linear limit (1/sqrt(3))
    constant TEST_ANGLES : real_array :=
        (0.0, 10.0, 30.0, 59.0, 61.0, 90.0, 119.0, 150.0, 179.0, 210.0, 239.0, 270.0, 300.0, 330.0, 359.0);
    constant TEST_MAGS : real_array := (0.05, 0.3, 0.577);

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, DATA_WIDTH);
    end function;

begin

    dut : entity work.svpwm
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map ( clk => clk, rst_n => rst_n, i_valid => i_valid,
                   i_valpha => i_valpha, i_vbeta => i_vbeta,
                   o_valid => o_valid, o_duty_a => o_duty_a,
                   o_duty_b => o_duty_b, o_duty_c => o_duty_c );

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
        variable valpha_r, vbeta_r : real;
        variable va_eff, vb_eff, vc_eff : real;
        variable alpha_chk, beta_chk : real;
        variable exp_alpha, exp_beta, got_alpha, got_beta : signed(DATA_WIDTH-1 downto 0);
        variable err_a, err_b : integer;
        variable pass_count, fail_count : integer := 0;
        variable total : integer := 0;
    begin
        rst_n <= '0'; i_valid <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        for m in TEST_MAGS'range loop
            for k in TEST_ANGLES'range loop
                theta_rad := TEST_ANGLES(k) * MATH_PI / 180.0;
                valpha_r := TEST_MAGS(m) * cos(theta_rad);
                vbeta_r  := TEST_MAGS(m) * sin(theta_rad);

                wait until rising_edge(clk);
                i_valpha <= to_q15(valpha_r);
                i_vbeta  <= to_q15(vbeta_r);
                i_valid  <= '1';
                wait until rising_edge(clk);
                i_valid  <= '0';
                wait until o_valid = '1';

                -- reconstruct effective phase voltages from the duty cycles
                va_eff := 2.0*(real(to_integer(o_duty_a))/65536.0) - 1.0;
                vb_eff := 2.0*(real(to_integer(o_duty_b))/65536.0) - 1.0;
                vc_eff := 2.0*(real(to_integer(o_duty_c))/65536.0) - 1.0;

                alpha_chk := (2.0*va_eff - vb_eff - vc_eff) / 3.0;
                beta_chk  := (vb_eff - vc_eff) / sqrt(3.0);

                exp_alpha := to_q15(valpha_r);
                exp_beta  := to_q15(vbeta_r);
                got_alpha := to_q15(alpha_chk);
                got_beta  := to_q15(beta_chk);

                err_a := abs(to_integer(got_alpha) - to_integer(exp_alpha));
                err_b := abs(to_integer(got_beta)  - to_integer(exp_beta));
                total := total + 1;

                if err_a <= TOL_LSB and err_b <= TOL_LSB then
                    pass_count := pass_count + 1;
                else
                    fail_count := fail_count + 1;
                    report "FAIL  mag=" & real'image(TEST_MAGS(m)) &
                           " theta=" & real'image(TEST_ANGLES(k)) &
                           "  err_a=" & integer'image(err_a) &
                           " err_b=" & integer'image(err_b) &
                           "  duty(a,b,c)=(" & integer'image(to_integer(o_duty_a)) & "," &
                           integer'image(to_integer(o_duty_b)) & "," &
                           integer'image(to_integer(o_duty_c)) & ")"
                        severity error;
                end if;

                wait for CLK_PERIOD * 2;
            end loop;
        end loop;

        report "================================================";
        report "SVPWM reconstruction test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(total);
        report "================================================";

        assert fail_count = 0
            report "SVPWM TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
