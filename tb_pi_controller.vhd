--------------------------------------------------------------------------------
-- tb_pi_controller.vhd
--
-- Three checks:
--  1) Step response for a constant, non-saturating error against the
--     closed-form result y(n) = Kp*E + n*Ki*E (valid once e_prev has
--     settled to E, i.e. from the 2nd sample onward).
--  2) Saturation: a large constant error should drive the output to
--     clip cleanly at the positive rail and stay there across multiple
--     samples (no wraparound).
--  3) Anti-windup recovery: immediately after saturating positive, a
--     reversed error should move the output measurably away from the
--     rail on the very next sample -- a windup bug would instead keep
--     it pinned near the rail for many samples.
--
-- Per the recursive-dependency note in pi_controller.vhd, each i_valid
-- pulse here waits for the previous o_valid before proceeding.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity tb_pi_controller is
end entity tb_pi_controller;

architecture sim of tb_pi_controller is

    constant DATA_WIDTH : integer := 16;
    constant GAIN_WIDTH : integer := 16;
    constant GAIN_FRAC  : integer := 11;
    constant CLK_PERIOD : time    := 10 ns;
    constant Q15_SCALE  : real    := 32768.0;
    constant Q11_SCALE  : real    := 2048.0;
    constant TOL_LSB    : integer := 4;

    signal clk     : std_logic := '0';
    signal rst_n   : std_logic := '0';
    signal i_valid : std_logic := '0';
    signal i_error : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal i_kp, i_ki : signed(GAIN_WIDTH-1 downto 0) := (others => '0');
    signal o_valid : std_logic;
    signal o_output : signed(DATA_WIDTH-1 downto 0);

    signal sim_done : boolean := false;

    function to_q15 (x : real) return signed is
        variable v : integer;
    begin
        v := integer(round(x * Q15_SCALE));
        if v > 32767 then v := 32767; end if;
        if v < -32768 then v := -32768; end if;
        return to_signed(v, DATA_WIDTH);
    end function;

    function to_q11 (x : real) return signed is
    begin
        return to_signed(integer(round(x * Q11_SCALE)), GAIN_WIDTH);
    end function;

    -- run one sample through the DUT, waiting properly for the recursive result
    procedure run_sample (
        signal clk_s     : in  std_logic;
        signal ivalid_s  : out std_logic;
        signal ierr_s    : out signed(DATA_WIDTH-1 downto 0);
        err_r            : in  real;
        signal ovalid_s  : in  std_logic;
        signal oout_s    : in  signed(DATA_WIDTH-1 downto 0);
        variable got     : out integer
    ) is
    begin
        wait until rising_edge(clk_s);
        ierr_s   <= to_q15(err_r);
        ivalid_s <= '1';
        wait until rising_edge(clk_s);
        ivalid_s <= '0';
        wait until ovalid_s = '1';
        got := to_integer(oout_s);
        wait for CLK_PERIOD;
    end procedure;

begin

    dut : entity work.pi_controller
        generic map ( DATA_WIDTH => DATA_WIDTH, GAIN_WIDTH => GAIN_WIDTH, GAIN_FRAC => GAIN_FRAC )
        port map ( clk => clk, rst_n => rst_n, i_valid => i_valid,
                   i_error => i_error, i_kp => i_kp, i_ki => i_ki,
                   o_valid => o_valid, o_output => o_output );

    clk_gen : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD/2;
            clk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    stim : process
        variable got : integer;
        variable exp_r : real;
        variable exp_q : integer;
        variable err_v : integer;
        variable pass_count, fail_count : integer := 0;
        constant KP1 : real := 1.0;
        constant KI1 : real := 0.05;
        constant E1  : real := 0.3;
        variable kp1_q, ki1_q : real;   -- gains as actually quantized to Q4.11 (matches hardware)
    begin
        rst_n <= '0'; i_valid <= '0';
        wait for CLK_PERIOD * 5;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        -----------------------------------------------------------------
        -- 1) Step response, non-saturating: y(n) = Kp*E + n*Ki*E
        -----------------------------------------------------------------
        i_kp <= to_q11(KP1);
        i_ki <= to_q11(KI1);
        kp1_q := real(to_integer(to_q11(KP1))) / Q11_SCALE;
        ki1_q := real(to_integer(to_q11(KI1))) / Q11_SCALE;

        for n in 1 to 5 loop
            run_sample(clk, i_valid, i_error, E1, o_valid, o_output, got);
            exp_r := kp1_q*E1 + real(n)*ki1_q*E1;
            exp_q := to_integer(to_q15(exp_r));
            err_v := abs(got - exp_q);
            if err_v <= TOL_LSB then
                pass_count := pass_count + 1;
                report "PASS  step n=" & integer'image(n) & "  err=" & integer'image(err_v);
            else
                fail_count := fail_count + 1;
                report "FAIL  step n=" & integer'image(n) & "  got=" & integer'image(got) &
                       " exp=" & integer'image(exp_q) severity error;
            end if;
        end loop;

        -----------------------------------------------------------------
        -- 2) Saturation: large constant error, output should clip and
        --    stay clipped at the positive rail (32767) across samples
        -----------------------------------------------------------------
        rst_n <= '0'; i_valid <= '0';
        wait for CLK_PERIOD * 3;
        rst_n <= '1';
        wait for CLK_PERIOD * 2;

        i_kp <= to_q11(1.0);
        i_ki <= to_q11(0.5);

        for n in 1 to 4 loop
            run_sample(clk, i_valid, i_error, 0.9, o_valid, o_output, got);
            if got = 32767 then
                pass_count := pass_count + 1;
                report "PASS  saturation step n=" & integer'image(n) & "  clipped at +max";
            else
                fail_count := fail_count + 1;
                report "FAIL  saturation step n=" & integer'image(n) & "  got=" & integer'image(got) &
                       " expected 32767" severity error;
            end if;
        end loop;

        -----------------------------------------------------------------
        -- 3) Anti-windup recovery: reverse the error hard; output must
        --    move well away from the rail on the very next sample
        -----------------------------------------------------------------
        run_sample(clk, i_valid, i_error, -0.9, o_valid, o_output, got);
        if got < 20000 then    -- moved meaningfully off the +32767 rail
            pass_count := pass_count + 1;
            report "PASS  anti-windup recovery: output moved to " & integer'image(got) &
                   " immediately after error reversal";
        else
            fail_count := fail_count + 1;
            report "FAIL  anti-windup recovery: output stuck near rail at " & integer'image(got)
                severity error;
        end if;

        report "================================================";
        report "PI controller test: " & integer'image(pass_count) &
               " passed, " & integer'image(fail_count) & " failed";
        report "================================================";

        assert fail_count = 0
            report "PI CONTROLLER TESTBENCH FAILED"
            severity failure;

        sim_done <= true;
        wait;
    end process;

end architecture sim;
