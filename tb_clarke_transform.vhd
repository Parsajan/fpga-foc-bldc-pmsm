--------------------------------------------------------------------------------
-- tb_clarke_transform.vhd
--
-- Self-checking testbench for clarke_transform.vhd.
--
-- Drives balanced sinusoidal phase currents ia=A*cos(theta),
-- ib=A*cos(theta-120deg) at a sweep of test angles, and checks the
-- output against the known closed-form result for an amplitude-invariant
-- Clarke transform under balanced conditions:
--
--   i_alpha = A*cos(theta)
--   i_beta  = A*sin(theta)
--
-- A tolerance of a few LSBs is allowed for Q1.15 quantization error.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_clarke_transform is
end entity tb_clarke_transform;

architecture sim of tb_clarke_transform is

    constant DATA_WIDTH : integer := 16;
    constant CLK_PERIOD : time    := 10 ns;    -- 100 MHz
    constant AMPLITUDE  : real    := 0.9;      -- stay inside Q1.15 range
    constant Q15_SCALE  : real    := 32768.0;
    constant TOL_LSB    : integer := 3;        -- allowed quantization error, in LSBs

    signal clk      : std_logic := '0';
    signal rst_n    : std_logic := '0';
    signal i_valid  : std_logic := '0';
    signal i_a, i_b : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal o_valid  : std_logic;
    signal o_alpha, o_beta : signed(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    type real_array is array (natural range <>) of real;
    constant TEST_ANGLES_DEG : real_array :=
        (0.0, 30.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 300.0, 359.0);

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, DATA_WIDTH);
    end function;

begin

    dut : entity work.clarke_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map (
            clk     => clk,
            rst_n   => rst_n,
            i_valid => i_valid,
            i_a     => i_a,
            i_b     => i_b,
            o_valid => o_valid,
            o_alpha => o_alpha,
            o_beta  => o_beta
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
        variable theta_rad  : real;
        variable ia_r, ib_r : real;
        variable exp_alpha_r, exp_beta_r : real;
        variable exp_alpha, exp_beta     : signed(DATA_WIDTH-1 downto 0);
        variable err_alpha, err_beta     : integer;
        variable pass_count, fail_count  : integer := 0;
    begin
        rst_n   <= '0';
        i_valid <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        for k in TEST_ANGLES_DEG'range loop
            theta_rad := TEST_ANGLES_DEG(k) * MATH_PI / 180.0;
            ia_r := AMPLITUDE * cos(theta_rad);
            ib_r := AMPLITUDE * cos(theta_rad - (2.0 * MATH_PI / 3.0));

            wait until rising_edge(clk);
            i_a     <= to_q15(ia_r);
            i_b     <= to_q15(ib_r);
            i_valid <= '1';
            wait until rising_edge(clk);
            i_valid <= '0';

            wait until o_valid = '1';

            exp_alpha_r := AMPLITUDE * cos(theta_rad);
            exp_beta_r  := AMPLITUDE * sin(theta_rad);
            exp_alpha := to_q15(exp_alpha_r);
            exp_beta  := to_q15(exp_beta_r);

            err_alpha := abs(to_integer(o_alpha) - to_integer(exp_alpha));
            err_beta  := abs(to_integer(o_beta)  - to_integer(exp_beta));

            if err_alpha <= TOL_LSB and err_beta <= TOL_LSB then
                pass_count := pass_count + 1;
                report "PASS  theta=" & real'image(TEST_ANGLES_DEG(k)) &
                       "deg  alpha_err=" & integer'image(err_alpha) &
                       " beta_err=" & integer'image(err_beta);
            else
                fail_count := fail_count + 1;
                report "FAIL  theta=" & real'image(TEST_ANGLES_DEG(k)) &
                       "deg  got(a,b)=(" & integer'image(to_integer(o_alpha)) & "," &
                       integer'image(to_integer(o_beta)) & ")  exp(a,b)=(" &
                       integer'image(to_integer(exp_alpha)) & "," &
                       integer'image(to_integer(exp_beta)) & ")"
                    severity error;
            end if;

            wait for CLK_PERIOD * 2;
        end loop;

        -- Deliberately violate the balanced-input assumption to exercise
        -- the beta saturation path: ia=+1, ib=+1 gives beta_sum=3,
        -- beta = 3/sqrt(3) = 1.732, which must clip to the max Q1.15 code.
        wait until rising_edge(clk);
        i_a     <= to_q15(0.99997);
        i_b     <= to_q15(0.99997);
        i_valid <= '1';
        wait until rising_edge(clk);
        i_valid <= '0';
        wait until o_valid = '1';
        if o_beta = to_signed(32767, DATA_WIDTH) then
            pass_count := pass_count + 1;
            report "PASS  saturation test: beta clipped to +max as expected";
        else
            fail_count := fail_count + 1;
            report "FAIL  saturation test: beta=" & integer'image(to_integer(o_beta)) &
                   ", expected clip to 32767"
                severity error;
        end if;

        report "================================================";
        report "Clarke transform test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(TEST_ANGLES_DEG'length + 1);
        report "================================================";

        assert fail_count = 0
            report "CLARKE TRANSFORM TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
