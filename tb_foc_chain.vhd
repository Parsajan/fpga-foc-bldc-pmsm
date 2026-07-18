--------------------------------------------------------------------------------
-- tb_foc_chain.vhd
--
-- Integration test: chains clarke_transform -> cordic_sincos ->
-- park_transform and checks the combined result against the textbook
-- closed-form Id/Iq computed directly from raw phase currents (ia, ib)
-- and rotor angle theta -- exercising the actual module interfaces
-- together, not just each block's math in isolation.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_foc_chain is
end entity tb_foc_chain;

architecture sim of tb_foc_chain is

    constant DATA_WIDTH  : integer := 16;
    constant ANGLE_WIDTH : integer := 22;
    constant CLK_PERIOD  : time    := 10 ns;
    constant Q15_SCALE   : real    := 32768.0;
    constant Q18_SCALE   : real    := 262144.0;
    constant TOL_LSB     : integer := 6;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    -- Clarke
    signal cl_valid_in : std_logic := '0';
    signal ia, ib       : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal cl_valid_out : std_logic;
    signal i_alpha, i_beta : signed(DATA_WIDTH-1 downto 0);

    -- CORDIC
    signal cd_valid_in : std_logic := '0';
    signal theta_q18    : signed(ANGLE_WIDTH-1 downto 0) := (others => '0');
    signal cd_busy, cd_valid_out : std_logic;
    signal cos_th, sin_th : signed(DATA_WIDTH-1 downto 0);

    -- Park
    signal pk_valid_in : std_logic := '0';
    signal pk_valid_out : std_logic;
    signal o_d, o_q : signed(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    type test_case_t is record
        ia_r, ib_r : real;
        theta_deg  : real;
    end record;
    type test_array_t is array (natural range <>) of test_case_t;
    constant TESTS : test_array_t := (
        (0.9,  0.0,   0.0),
        (0.9, -0.45,  90.0),
        (0.3,  0.5,  200.0),
        (-0.6, 0.2, -150.0),
        (0.8, -0.8,   33.0)
    );

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, DATA_WIDTH);
    end function;

    function to_q18 (x_rad : real) return signed is
    begin
        return to_signed(integer(round(x_rad * Q18_SCALE)), ANGLE_WIDTH);
    end function;

begin

    u_clarke : entity work.clarke_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map ( clk => clk, rst_n => rst_n, i_valid => cl_valid_in,
                   i_a => ia, i_b => ib,
                   o_valid => cl_valid_out, o_alpha => i_alpha, o_beta => i_beta );

    u_cordic : entity work.cordic_sincos
        port map ( clk => clk, rst_n => rst_n, i_valid => cd_valid_in,
                   i_theta => theta_q18,
                   o_busy => cd_busy, o_valid => cd_valid_out,
                   o_cos => cos_th, o_sin => sin_th );

    u_park : entity work.park_transform
        generic map ( DATA_WIDTH => DATA_WIDTH )
        port map ( clk => clk, rst_n => rst_n, i_valid => pk_valid_in,
                   i_alpha => i_alpha, i_beta => i_beta,
                   i_cos => cos_th, i_sin => sin_th,
                   o_valid => pk_valid_out, o_d => o_d, o_q => o_q );

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
        variable exp_alpha, exp_beta : real;
        variable exp_d_r, exp_q_r : real;
        variable exp_d, exp_q : signed(DATA_WIDTH-1 downto 0);
        variable err_d, err_q : integer;
        variable pass_count, fail_count : integer := 0;
    begin
        rst_n <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        for k in TESTS'range loop
            theta_rad := TESTS(k).theta_deg * MATH_PI / 180.0;

            -- kick off Clarke and CORDIC together
            wait until rising_edge(clk);
            ia <= to_q15(TESTS(k).ia_r);
            ib <= to_q15(TESTS(k).ib_r);
            cl_valid_in <= '1';
            theta_q18 <= to_q18(theta_rad);
            cd_valid_in <= '1';
            wait until rising_edge(clk);
            cl_valid_in <= '0';
            cd_valid_in <= '0';

            -- wait for both Clarke and CORDIC outputs to be ready
            wait until cl_valid_out = '1';
            wait until cd_valid_out = '1';

            -- feed Park on the next clock edge
            wait until rising_edge(clk);
            pk_valid_in <= '1';
            wait until rising_edge(clk);
            pk_valid_in <= '0';

            wait until pk_valid_out = '1';

            exp_alpha := TESTS(k).ia_r;
            exp_beta  := (TESTS(k).ia_r + 2.0*TESTS(k).ib_r) / sqrt(3.0);
            exp_d_r := exp_alpha*cos(theta_rad) + exp_beta*sin(theta_rad);
            exp_q_r := -exp_alpha*sin(theta_rad) + exp_beta*cos(theta_rad);
            exp_d := to_q15(exp_d_r);
            exp_q := to_q15(exp_q_r);

            err_d := abs(to_integer(o_d) - to_integer(exp_d));
            err_q := abs(to_integer(o_q) - to_integer(exp_q));

            if err_d <= TOL_LSB and err_q <= TOL_LSB then
                pass_count := pass_count + 1;
                report "PASS  case=" & integer'image(k) &
                       "  d_err=" & integer'image(err_d) &
                       " q_err=" & integer'image(err_q);
            else
                fail_count := fail_count + 1;
                report "FAIL  case=" & integer'image(k) &
                       "  got(d,q)=(" & integer'image(to_integer(o_d)) & "," &
                       integer'image(to_integer(o_q)) & ")  exp(d,q)=(" &
                       integer'image(to_integer(exp_d)) & "," &
                       integer'image(to_integer(exp_q)) & ")"
                    severity error;
            end if;

            wait for CLK_PERIOD * 3;
        end loop;

        report "================================================";
        report "FOC chain (Clarke->CORDIC->Park) test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) &
               " failed, out of " & integer'image(TESTS'length);
        report "================================================";

        assert fail_count = 0
            report "FOC CHAIN INTEGRATION TEST FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
