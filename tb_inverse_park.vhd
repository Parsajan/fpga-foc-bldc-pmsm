--------------------------------------------------------------------------------
-- tb_inverse_park.vhd
--
-- Two checks:
--  1) Direct: v_alpha/v_beta against the closed-form R(-theta) rotation
--     of (Vd,Vq), for cos/sin supplied directly via math_real.
--  2) Round-trip: feed this module's (v_alpha,v_beta) output into the
--     already-verified park_transform with the same cos/sin, and check
--     we recover the original (Vd,Vq) -- confirms the two modules are
--     genuine inverses of one another, not just individually plausible.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_inverse_park is
end entity tb_inverse_park;

architecture sim of tb_inverse_park is

    constant DATA_WIDTH : integer := 16;
    constant CLK_PERIOD : time    := 10 ns;
    constant Q15_SCALE  : real    := 32768.0;
    constant TOL_LSB    : integer := 3;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';

    signal ip_valid_in : std_logic := '0';
    signal vd, vq, cos_th, sin_th : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal ip_valid_out : std_logic;
    signal v_alpha, v_beta : signed(DATA_WIDTH-1 downto 0);

    signal pk_valid_in : std_logic := '0';
    signal pk_valid_out : std_logic;
    signal d_rt, q_rt : signed(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    type test_case_t is record
        d_r, q_r, theta_deg : real;
    end record;
    type test_array_t is array (natural range <>) of test_case_t;
    constant TESTS : test_array_t := (
        (0.9,  0.0,   0.0),
        (0.0,  0.9,  45.0),
        (0.6, -0.6,  90.0),
        (-0.5, 0.3, 150.0),
        (0.4,  0.4, -60.0),
        (-0.7,-0.5, 200.0),
        (0.99, 0.05,  10.0)
    );

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, DATA_WIDTH);
    end function;

begin

    u_inv_park : entity work.inverse_park_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map ( clk => clk, rst_n => rst_n, i_valid => ip_valid_in,
                   i_d => vd, i_q => vq, i_cos => cos_th, i_sin => sin_th,
                   o_valid => ip_valid_out, o_alpha => v_alpha, o_beta => v_beta );

    u_park : entity work.park_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map ( clk => clk, rst_n => rst_n, i_valid => pk_valid_in,
                   i_alpha => v_alpha, i_beta => v_beta,
                   i_cos => cos_th, i_sin => sin_th,
                   o_valid => pk_valid_out, o_d => d_rt, o_q => q_rt );

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
        variable c, s : real;
        variable exp_a_r, exp_b_r : real;
        variable exp_a, exp_b : signed(DATA_WIDTH-1 downto 0);
        variable err_a, err_b, err_d, err_q : integer;
        variable pass_count, fail_count : integer := 0;
    begin
        rst_n <= '0'; wait for CLK_PERIOD * 5;
        rst_n <= '1'; wait for CLK_PERIOD * 2;

        for k in TESTS'range loop
            theta_rad := TESTS(k).theta_deg * MATH_PI / 180.0;
            c := cos(theta_rad);
            s := sin(theta_rad);

            wait until rising_edge(clk);
            vd <= to_q15(TESTS(k).d_r);
            vq <= to_q15(TESTS(k).q_r);
            cos_th <= to_q15(c);
            sin_th <= to_q15(s);
            ip_valid_in <= '1';
            wait until rising_edge(clk);
            ip_valid_in <= '0';
            wait until ip_valid_out = '1';

            -- Check 1: direct closed-form
            exp_a_r := TESTS(k).d_r*c - TESTS(k).q_r*s;
            exp_b_r := TESTS(k).d_r*s + TESTS(k).q_r*c;
            exp_a := to_q15(exp_a_r);
            exp_b := to_q15(exp_b_r);
            err_a := abs(to_integer(v_alpha) - to_integer(exp_a));
            err_b := abs(to_integer(v_beta)  - to_integer(exp_b));

            -- Check 2: round-trip through the forward Park transform
            wait until rising_edge(clk);
            pk_valid_in <= '1';
            wait until rising_edge(clk);
            pk_valid_in <= '0';
            wait until pk_valid_out = '1';

            err_d := abs(to_integer(d_rt) - to_integer(to_q15(TESTS(k).d_r)));
            err_q := abs(to_integer(q_rt) - to_integer(to_q15(TESTS(k).q_r)));

            if err_a <= TOL_LSB and err_b <= TOL_LSB and err_d <= TOL_LSB and err_q <= TOL_LSB then
                pass_count := pass_count + 1;
                report "PASS  case=" & integer'image(k) &
                       "  direct_err=(" & integer'image(err_a) & "," & integer'image(err_b) & ")" &
                       "  roundtrip_err=(" & integer'image(err_d) & "," & integer'image(err_q) & ")";
            else
                fail_count := fail_count + 1;
                report "FAIL  case=" & integer'image(k) &
                       "  direct_err=(" & integer'image(err_a) & "," & integer'image(err_b) & ")" &
                       "  roundtrip_err=(" & integer'image(err_d) & "," & integer'image(err_q) & ")"
                    severity error;
            end if;

            wait for CLK_PERIOD * 3;
        end loop;

        report "================================================";
        report "Inverse Park test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(TESTS'length);
        report "================================================";

        assert fail_count = 0
            report "INVERSE PARK TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
