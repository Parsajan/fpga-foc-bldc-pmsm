--------------------------------------------------------------------------------
-- tb_park_transform.vhd
--
-- Self-checking testbench for park_transform.vhd, tested in isolation
-- (i_cos/i_sin supplied directly from math_real, not from the CORDIC
-- core, to test this module's own multiply/add/round/saturate logic
-- independently of CORDIC's small residual error).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_park_transform is
end entity tb_park_transform;

architecture sim of tb_park_transform is

    constant DATA_WIDTH : integer := 16;
    constant CLK_PERIOD : time    := 10 ns;
    constant Q15_SCALE  : real    := 32768.0;
    constant TOL_LSB    : integer := 2;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal i_valid : std_logic := '0';
    signal i_alpha, i_beta, i_cos, i_sin : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal o_valid : std_logic;
    signal o_d, o_q : signed(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    type test_case_t is record
        alpha_r   : real;
        beta_r    : real;
        theta_deg : real;
    end record;
    type test_array_t is array (natural range <>) of test_case_t;

    constant TESTS : test_array_t := (
        (0.9,  0.0,  0.0),
        (0.0,  0.9,  0.0),
        (0.6,  0.6,  45.0),
        (0.9, -0.5,  30.0),
        (-0.7, 0.3, 120.0),
        (0.5,  0.5, -60.0),
        (0.99, 0.0,  0.0),
        (0.0,  0.0,  45.0),
        (-0.8,-0.4, 200.0),
        (0.3, -0.9, -135.0),
        (0.99, 0.99, 45.0)
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

    dut : entity work.park_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map (
            clk => clk, rst_n => rst_n, i_valid => i_valid,
            i_alpha => i_alpha, i_beta => i_beta,
            i_cos => i_cos, i_sin => i_sin,
            o_valid => o_valid, o_d => o_d, o_q => o_q
        );

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
        variable exp_d_r, exp_q_r : real;
        variable exp_d, exp_q : signed(DATA_WIDTH-1 downto 0);
        variable err_d, err_q : integer;
        variable pass_count, fail_count : integer := 0;
    begin
        rst_n <= '0'; i_valid <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        for k in TESTS'range loop
            theta_rad := TESTS(k).theta_deg * MATH_PI / 180.0;
            c := cos(theta_rad);
            s := sin(theta_rad);

            wait until rising_edge(clk);
            i_alpha <= to_q15(TESTS(k).alpha_r);
            i_beta  <= to_q15(TESTS(k).beta_r);
            i_cos   <= to_q15(c);
            i_sin   <= to_q15(s);
            i_valid <= '1';
            wait until rising_edge(clk);
            i_valid <= '0';

            wait until o_valid = '1';

            exp_d_r := TESTS(k).alpha_r * c + TESTS(k).beta_r * s;
            exp_q_r := -TESTS(k).alpha_r * s + TESTS(k).beta_r * c;
            exp_d := to_q15(exp_d_r);
            exp_q := to_q15(exp_q_r);

            err_d := abs(to_integer(o_d) - to_integer(exp_d));
            err_q := abs(to_integer(o_q) - to_integer(exp_q));

            -- widen tolerance automatically for cases that clip (exp_d_r/exp_q_r
            -- outside +-1): rounding right at the saturation boundary can
            -- legitimately land 1-2 LSB differently depending on rounding order
            if (abs(exp_d_r) >= 0.9999 and err_d <= 4) or err_d <= TOL_LSB then
                if (abs(exp_q_r) >= 0.9999 and err_q <= 4) or err_q <= TOL_LSB then
                    pass_count := pass_count + 1;
                    report "PASS  case=" & integer'image(k) &
                           "  d_err=" & integer'image(err_d) &
                           " q_err=" & integer'image(err_q);
                else
                    fail_count := fail_count + 1;
                    report "FAIL(q) case=" & integer'image(k) &
                           "  got_q=" & integer'image(to_integer(o_q)) &
                           " exp_q=" & integer'image(to_integer(exp_q)) &
                           " (" & real'image(exp_q_r) & ")"
                        severity error;
                end if;
            else
                fail_count := fail_count + 1;
                report "FAIL(d) case=" & integer'image(k) &
                       "  got_d=" & integer'image(to_integer(o_d)) &
                       " exp_d=" & integer'image(to_integer(exp_d)) &
                       " (" & real'image(exp_d_r) & ")"
                    severity error;
            end if;

            wait for CLK_PERIOD * 2;
        end loop;

        report "================================================";
        report "Park transform test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(TESTS'length);
        report "================================================";

        assert fail_count = 0
            report "PARK TRANSFORM TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
