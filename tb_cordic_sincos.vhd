--------------------------------------------------------------------------------
-- tb_cordic_sincos.vhd
--
-- Self-checking testbench for cordic_sincos.vhd. Sweeps test angles
-- across the full [-pi, pi) range -- including points that exercise the
-- quadrant range-reduction logic -- and checks o_cos/o_sin against
-- math_real's cos()/sin(), within a tolerance for CORDIC + fixed-point
-- approximation error.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_cordic_sincos is
end entity tb_cordic_sincos;

architecture sim of tb_cordic_sincos is

    constant ANGLE_WIDTH : integer := 22;
    constant ANGLE_FRAC  : integer := 18;
    constant OUT_WIDTH   : integer := 16;
    constant CLK_PERIOD  : time    := 10 ns;
    constant Q15_SCALE   : real    := 32768.0;
    constant Q18_SCALE   : real    := 262144.0;   -- 2^18
    constant TOL_LSB      : integer := 6;           -- allowed error at Q1.15, in LSBs

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal i_valid : std_logic := '0';
    signal i_theta : signed(ANGLE_WIDTH-1 downto 0) := (others => '0');
    signal o_busy  : std_logic;
    signal o_valid : std_logic;
    signal o_cos, o_sin : signed(OUT_WIDTH-1 downto 0);

    signal sim_done : boolean := false;
    signal cycle_count : integer := 0;

    type real_array is array (natural range <>) of real;
    constant TEST_ANGLES_DEG : real_array :=
        (0.0, 15.0, 30.0, 45.0, 60.0, 89.9, 90.0, 90.1, 120.0, 135.0,
         150.0, 179.0, -179.0, -150.0, -90.1, -90.0, -89.9, -45.0, -1.0, 1.0);

    function to_q18 (x_rad : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x_rad * Q18_SCALE));
        return to_signed(v, ANGLE_WIDTH);
    end function;

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, OUT_WIDTH);
    end function;

begin

    dut : entity work.cordic_sincos
        port map (
            clk     => clk,
            rst_n   => rst_n,
            i_valid => i_valid,
            i_theta => i_theta,
            o_busy  => o_busy,
            o_valid => o_valid,
            o_cos   => o_cos,
            o_sin   => o_sin
        );

    clk_gen : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD/2;
            clk <= '1'; wait for CLK_PERIOD/2;
            cycle_count <= cycle_count + 1;
        end loop;
        wait;
    end process;

    stim : process
        variable theta_rad : real;
        variable exp_cos, exp_sin : signed(OUT_WIDTH-1 downto 0);
        variable err_cos, err_sin : integer;
        variable pass_count, fail_count : integer := 0;
        variable start_cycle, latency : integer;
    begin
        rst_n   <= '0';
        i_valid <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        for k in TEST_ANGLES_DEG'range loop
            theta_rad := TEST_ANGLES_DEG(k) * MATH_PI / 180.0;

            wait until rising_edge(clk);
            i_theta <= to_q18(theta_rad);
            i_valid <= '1';
            start_cycle := cycle_count;
            wait until rising_edge(clk);
            i_valid <= '0';

            wait until o_valid = '1';
            latency := cycle_count - start_cycle;

            exp_cos := to_q15(cos(theta_rad));
            exp_sin := to_q15(sin(theta_rad));

            err_cos := abs(to_integer(o_cos) - to_integer(exp_cos));
            err_sin := abs(to_integer(o_sin) - to_integer(exp_sin));

            if err_cos <= TOL_LSB and err_sin <= TOL_LSB then
                pass_count := pass_count + 1;
                report "PASS  theta=" & real'image(TEST_ANGLES_DEG(k)) &
                       "deg  cos_err=" & integer'image(err_cos) &
                       " sin_err=" & integer'image(err_sin) &
                       " latency=" & integer'image(latency) & "cyc";
            else
                fail_count := fail_count + 1;
                report "FAIL  theta=" & real'image(TEST_ANGLES_DEG(k)) &
                       "deg  got(c,s)=(" & integer'image(to_integer(o_cos)) & "," &
                       integer'image(to_integer(o_sin)) & ")  exp(c,s)=(" &
                       integer'image(to_integer(exp_cos)) & "," &
                       integer'image(to_integer(exp_sin)) & ")"
                    severity error;
            end if;

            wait for CLK_PERIOD * 3;
        end loop;

        report "================================================";
        report "CORDIC sin/cos test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(TEST_ANGLES_DEG'length);
        report "================================================";

        assert fail_count = 0
            report "CORDIC TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
