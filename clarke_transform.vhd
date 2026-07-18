--------------------------------------------------------------------------------
-- clarke_transform.vhd
--
-- Amplitude-invariant Clarke transform (2-input form), converting two
-- measured phase currents into the stationary alpha/beta frame:
--
--   i_alpha = i_a
--   i_beta  = (i_a + 2*i_b) / sqrt(3)
--
-- Assumes a balanced three-phase system (i_a + i_b + i_c = 0), so only
-- two phase currents need to be sensed (standard practice; the third is
-- reconstructed implicitly).
--
-- Fixed-point format: Q1.15 signed (16-bit), representing the range
-- [-1, 1). Fully pipelined, 3 clock cycles of latency, one new sample
-- accepted per clock cycle.
--
-- i_alpha is a pure passthrough/resize of i_a, so it can never overflow.
-- i_beta involves a multiply-and-rescale, so it is saturated on output;
-- for balanced sinusoidal inputs within +-1 this never triggers, but it
-- protects against sensor faults or startup transients that could
-- momentarily violate the balanced-input assumption.
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clarke_transform is
    generic (
        DATA_WIDTH : integer := 16     -- Q1.15
    );
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;                             -- active-low, synchronous
        i_valid : in  std_logic;
        i_a     : in  signed(DATA_WIDTH-1 downto 0);         -- Q1.15
        i_b     : in  signed(DATA_WIDTH-1 downto 0);         -- Q1.15
        o_valid : out std_logic;
        o_alpha : out signed(DATA_WIDTH-1 downto 0);         -- Q1.15
        o_beta  : out signed(DATA_WIDTH-1 downto 0)          -- Q1.15
    );
end entity clarke_transform;

architecture rtl of clarke_transform is

    constant FRAC_BITS : integer := DATA_WIDTH - 1;          -- 15
    constant SUM_WIDTH  : integer := DATA_WIDTH + 2;         -- 18: headroom for (ia + 2*ib) in [-4,4)
    constant MUL_WIDTH  : integer := SUM_WIDTH + DATA_WIDTH; -- 34
    constant SHR_WIDTH  : integer := MUL_WIDTH - FRAC_BITS;  -- 19

    -- 1/sqrt(3) = 0.5773502691896258, in Q1.15: round(0.5773502691896258 * 2^15) = 18920
    constant ONE_OVER_SQRT3 : signed(DATA_WIDTH-1 downto 0) := to_signed(18920, DATA_WIDTH);

    -- Stage 1
    signal alpha_s1    : signed(DATA_WIDTH-1 downto 0);
    signal beta_sum_s1 : signed(SUM_WIDTH-1 downto 0);
    signal valid_s1    : std_logic;

    -- Stage 2
    signal alpha_s2    : signed(DATA_WIDTH-1 downto 0);
    signal beta_mul_s2 : signed(MUL_WIDTH-1 downto 0);
    signal valid_s2    : std_logic;

    -- Stage 3 combinational slice of the rescaled product, before saturation
    signal beta_shifted : signed(SHR_WIDTH-1 downto 0);

    -- Saturate a (DATA_WIDTH+3)-bit signed value down to DATA_WIDTH bits.
    function saturate19 (w : signed(SHR_WIDTH-1 downto 0)) return signed is
        variable result : signed(DATA_WIDTH-1 downto 0);
    begin
        if (w(SHR_WIDTH-1) = w(SHR_WIDTH-2)) and (w(SHR_WIDTH-2) = w(SHR_WIDTH-3))
           and (w(SHR_WIDTH-3) = w(DATA_WIDTH-1)) then
            result := w(DATA_WIDTH-1 downto 0);           -- fits, no overflow
        elsif w(SHR_WIDTH-1) = '1' then
            result := to_signed(-(2**(DATA_WIDTH-1)), DATA_WIDTH);   -- most negative
        else
            result := to_signed(2**(DATA_WIDTH-1) - 1, DATA_WIDTH); -- most positive
        end if;
        return result;
    end function;

begin

    ----------------------------------------------------------------------
    -- Stage 1: alpha passthrough, beta_sum = ia + 2*ib (widened, no overflow)
    ----------------------------------------------------------------------
    process (clk)
        variable ia_ext, ib_ext : signed(SUM_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                alpha_s1    <= (others => '0');
                beta_sum_s1 <= (others => '0');
                valid_s1    <= '0';
            else
                ia_ext := resize(i_a, SUM_WIDTH);
                ib_ext := resize(i_b, SUM_WIDTH);
                alpha_s1    <= i_a;
                beta_sum_s1 <= ia_ext + ib_ext + ib_ext;      -- ia + 2*ib
                valid_s1    <= i_valid;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 2: alpha piped through, beta_sum * (1/sqrt3)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                alpha_s2    <= (others => '0');
                beta_mul_s2 <= (others => '0');
                valid_s2    <= '0';
            else
                alpha_s2    <= alpha_s1;
                beta_mul_s2 <= beta_sum_s1 * ONE_OVER_SQRT3;
                valid_s2    <= valid_s1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 3: rescale beta (>>15 to undo the Q1.15 coefficient scaling)
    -- and saturate to Q1.15 output width. Alpha needs no saturation: it
    -- was only ever resized, never combined, so truncation back to
    -- DATA_WIDTH is always exact.
    ----------------------------------------------------------------------
    beta_shifted <= beta_mul_s2(MUL_WIDTH-1 downto FRAC_BITS);

    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                o_alpha <= (others => '0');
                o_beta  <= (others => '0');
                o_valid <= '0';
            else
                o_alpha <= alpha_s2;
                o_beta  <= saturate19(beta_shifted);
                o_valid <= valid_s2;
            end if;
        end if;
    end process;

end architecture rtl;
