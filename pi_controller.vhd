--------------------------------------------------------------------------------
-- pi_controller.vhd
--
-- Incremental (velocity-form) PI controller for a single d- or q-axis
-- current loop:
--
--   delta_y(n) = Kp*(e(n) - e(n-1)) + Ki*e(n)
--   y(n)       = sat(y(n-1) + delta_y(n))
--
-- The incremental form has no separate integral accumulator to wind up:
-- the only state is the previous (already-saturated) output y(n-1), so
-- clamping y(n) at the output automatically clamps the state used next
-- cycle -- standard "clamping" anti-windup, for free, by construction.
--
-- IMPORTANT -- recursive dependency: unlike the feedforward Clarke/Park/
-- CORDIC blocks, y(n) depends on y(n-1), so a new i_valid must not be
-- asserted until o_valid has returned for the previous sample (this
-- module is not safe to pipeline back-to-back the way the feedforward
-- blocks are). In practice this is a non-issue: a 20 kHz current loop on
-- a 100 MHz clock leaves ~5000 cycles between samples, versus 5 cycles
-- of latency here.
--
-- Formats:
--   i_error, o_output : Q1.15 signed, 16-bit
--   i_kp, i_ki         : Q4.11 signed, 16-bit (runtime-adjustable gains)
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pi_controller is
    generic (
        DATA_WIDTH : integer := 16;   -- Q1.15
        GAIN_WIDTH : integer := 16;   -- Q4.11
        GAIN_FRAC  : integer := 11
    );
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic;
        i_valid  : in  std_logic;
        i_error  : in  signed(DATA_WIDTH-1 downto 0);   -- e(n) = reference - measured
        i_kp     : in  signed(GAIN_WIDTH-1 downto 0);
        i_ki     : in  signed(GAIN_WIDTH-1 downto 0);
        o_valid  : out std_logic;
        o_output : out signed(DATA_WIDTH-1 downto 0)
    );
end entity pi_controller;

architecture rtl of pi_controller is

    constant DATA_FRAC : integer := DATA_WIDTH - 1;                    -- 15
    constant DE_WIDTH   : integer := DATA_WIDTH + 1;                    -- 17, Q2.15
    constant T1_WIDTH   : integer := GAIN_WIDTH + DE_WIDTH;             -- 33, Q6.26
    constant T2_WIDTH   : integer := GAIN_WIDTH + DATA_WIDTH;           -- 32, Q5.26
    constant SUM_WIDTH  : integer := T1_WIDTH + 1;                      -- 34, Q7.26 (T1 is the wider operand)
    constant RS_SHIFT    : integer := GAIN_FRAC;                         -- 11: frac bits to drop, Q?.26 -> Q?.15
    constant WIDE_WIDTH  : integer := SUM_WIDTH - RS_SHIFT + 1;          -- 24: delta, rescaled, headroom kept (no sat yet)

    -- Pipeline registers
    signal e_prev_reg : signed(DATA_WIDTH-1 downto 0) := (others => '0');
    signal y_prev_reg : signed(DATA_WIDTH-1 downto 0) := (others => '0');

    signal delta_e_s1, e_s1 : signed(DATA_WIDTH downto 0);   -- DE_WIDTH-1 downto 0, i.e. 17 bits (indices 16 downto 0)
    signal kp_s1, ki_s1 : signed(GAIN_WIDTH-1 downto 0);
    signal valid_s1 : std_logic;

    signal term1_s2, term2_s2 : signed(T1_WIDTH-1 downto 0);  -- term2 resized into same container, upper bits unused-but-safe
    signal valid_s2 : std_logic;

    signal sum_s3 : signed(SUM_WIDTH-1 downto 0);
    signal valid_s3 : std_logic;

    signal wide_delta_s4 : signed(WIDE_WIDTH-1 downto 0);
    signal valid_s4 : std_logic;

    -- Round-to-nearest a Q?.26 value down to Q?.15, keeping full headroom (no saturation)
    function round_wide (w : signed(SUM_WIDTH-1 downto 0)) return signed is
        variable rounded : signed(SUM_WIDTH downto 0);
    begin
        rounded := resize(w, SUM_WIDTH+1) + to_signed(2**(RS_SHIFT-1), SUM_WIDTH+1);
        return resize(rounded(SUM_WIDTH downto RS_SHIFT), WIDE_WIDTH);
    end function;

    -- Saturate a WIDE_WIDTH-bit signed value down to Q1.(DATA_WIDTH-1)
    function saturate (w : signed(WIDE_WIDTH-1 downto 0)) return signed is
        variable result : signed(DATA_WIDTH-1 downto 0);
        variable safe   : boolean;
    begin
        safe := true;
        for b in DATA_WIDTH-1 to WIDE_WIDTH-2 loop
            if w(b+1) /= w(b) then
                safe := false;
            end if;
        end loop;

        if safe then
            result := w(DATA_WIDTH-1 downto 0);
        elsif w(WIDE_WIDTH-1) = '1' then
            result := to_signed(-(2**(DATA_WIDTH-1)), DATA_WIDTH);
        else
            result := to_signed(2**(DATA_WIDTH-1)-1, DATA_WIDTH);
        end if;
        return result;
    end function;

begin

    ----------------------------------------------------------------------
    -- Stage 1: delta_e = e(n) - e(n-1); latch gains; update e_prev for
    -- the *next* sample (uses the OLD e_prev_reg for this computation).
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                delta_e_s1 <= (others => '0');
                e_s1       <= (others => '0');
                kp_s1      <= (others => '0');
                ki_s1      <= (others => '0');
                valid_s1   <= '0';
                e_prev_reg <= (others => '0');
            else
                if i_valid = '1' then
                    delta_e_s1 <= resize(i_error, DATA_WIDTH+1) - resize(e_prev_reg, DATA_WIDTH+1);
                    e_s1       <= resize(i_error, DATA_WIDTH+1);
                    kp_s1      <= i_kp;
                    ki_s1      <= i_ki;
                    e_prev_reg <= i_error;
                end if;
                valid_s1 <= i_valid;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 2: term1 = Kp * delta_e (Q6.26), term2 = Ki * e(n) (Q5.26,
    -- stored in the same-width container as term1 for a simple add next)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                term1_s2 <= (others => '0');
                term2_s2 <= (others => '0');
                valid_s2 <= '0';
            else
                term1_s2 <= kp_s1 * delta_e_s1;
                term2_s2 <= resize(ki_s1 * e_s1(DATA_WIDTH-1 downto 0), T1_WIDTH);
                valid_s2 <= valid_s1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 3: sum = term1 + term2 (Q7.26)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                sum_s3   <= (others => '0');
                valid_s3 <= '0';
            else
                sum_s3   <= resize(term1_s2, SUM_WIDTH) + resize(term2_s2, SUM_WIDTH);
                valid_s3 <= valid_s2;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 4: rescale to a wide Q_.15-equivalent delta (no saturation yet)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                wide_delta_s4 <= (others => '0');
                valid_s4      <= '0';
            else
                wide_delta_s4 <= round_wide(sum_s3);
                valid_s4      <= valid_s3;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 5: y(n) = sat(y(n-1) + delta); this saturated value becomes
    -- both the output and the new y_prev_reg (clamping anti-windup).
    ----------------------------------------------------------------------
    process (clk)
        variable y_new_wide : signed(WIDE_WIDTH-1 downto 0);
        variable y_sat      : signed(DATA_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                y_prev_reg <= (others => '0');
                o_output   <= (others => '0');
                o_valid    <= '0';
            else
                if valid_s4 = '1' then
                    y_new_wide := resize(y_prev_reg, WIDE_WIDTH) + wide_delta_s4;
                    y_sat := saturate(y_new_wide);
                    y_prev_reg <= y_sat;
                    o_output   <= y_sat;
                end if;
                o_valid <= valid_s4;
            end if;
        end if;
    end process;

end architecture rtl;
