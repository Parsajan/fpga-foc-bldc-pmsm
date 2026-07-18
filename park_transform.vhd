--------------------------------------------------------------------------------
-- park_transform.vhd
--
-- Park transform: rotates the stationary alpha/beta frame into the
-- rotor-synchronous d/q frame using the electrical angle theta:
--
--   i_d =  i_alpha*cos(theta) + i_beta*sin(theta)
--   i_q = -i_alpha*sin(theta) + i_beta*cos(theta)
--
-- Takes cos(theta)/sin(theta) as inputs (from cordic_sincos.vhd) rather
-- than theta itself, so this module has no trigonometry of its own --
-- just 4 multiplies and 2 adds. (A 3-multiplier algebraic reformulation
-- exists; the direct 4-multiplier form is used here for clarity.)
--
-- All ports Q1.15 signed (16-bit). Fully pipelined, 3 cycles latency,
-- one sample per clock throughput.
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity park_transform is
    generic (
        DATA_WIDTH : integer := 16    -- Q1.15
    );
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;
        i_valid : in  std_logic;
        i_alpha : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15
        i_beta  : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15
        i_cos   : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, cos(theta)
        i_sin   : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, sin(theta)
        o_valid : out std_logic;
        o_d     : out signed(DATA_WIDTH-1 downto 0);   -- Q1.15
        o_q     : out signed(DATA_WIDTH-1 downto 0)    -- Q1.15
    );
end entity park_transform;

architecture rtl of park_transform is

    constant SUM_FRAC    : integer := 2 * (DATA_WIDTH - 1);        -- 30: fractional bits in the Q3.30 sum
    constant OUT_FRAC    : integer := DATA_WIDTH - 1;              -- 15: fractional bits in the Q1.15 output
    constant PROD_WIDTH  : integer := 2 * DATA_WIDTH;              -- 32, Q2.30
    constant SUM_WIDTH   : integer := PROD_WIDTH + 1;              -- 33, Q3.30 (1 headroom bit)
    constant SHIFT_WIDTH : integer := SUM_WIDTH - (SUM_FRAC - OUT_FRAC) + 1;  -- 19, slice width after dropping frac bits

    -- Stage 1: registered products
    signal p_ac_s1, p_bs_s1, p_as_s1, p_bc_s1 : signed(PROD_WIDTH-1 downto 0);
    signal valid_s1 : std_logic;

    -- Stage 2: registered sums (d = ac+bs, q = bc-as)
    signal sum_d_s2, sum_q_s2 : signed(SUM_WIDTH-1 downto 0);
    signal valid_s2 : std_logic;

    -- Round-to-nearest and saturate a Q3.30 value to Q1.15
    function round_sat (w : signed(SUM_WIDTH-1 downto 0)) return signed is
        constant SHIFT : integer := SUM_FRAC - OUT_FRAC;   -- 15: fractional bits to drop
        variable rounded : signed(SUM_WIDTH downto 0);
        variable slice1   : signed(SHIFT_WIDTH-1 downto 0);
        variable result   : signed(DATA_WIDTH-1 downto 0);
        variable safe     : boolean;
    begin
        if SHIFT = 0 then
            rounded := resize(w, SUM_WIDTH+1);
        else
            rounded := resize(w, SUM_WIDTH+1) + to_signed(2**(SHIFT-1), SUM_WIDTH+1);
        end if;
        slice1 := rounded(SUM_WIDTH downto SHIFT);

        safe := true;
        for b in DATA_WIDTH-1 to SHIFT_WIDTH-2 loop
            if slice1(b+1) /= slice1(b) then
                safe := false;
            end if;
        end loop;

        if safe then
            result := slice1(DATA_WIDTH-1 downto 0);
        elsif slice1(SHIFT_WIDTH-1) = '1' then
            result := to_signed(-(2**(DATA_WIDTH-1)), DATA_WIDTH);
        else
            result := to_signed(2**(DATA_WIDTH-1)-1, DATA_WIDTH);
        end if;
        return result;
    end function;

begin

    ----------------------------------------------------------------------
    -- Stage 1: four products (Ialpha*cos, Ibeta*sin, Ialpha*sin, Ibeta*cos)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                p_ac_s1  <= (others => '0');
                p_bs_s1  <= (others => '0');
                p_as_s1  <= (others => '0');
                p_bc_s1  <= (others => '0');
                valid_s1 <= '0';
            else
                p_ac_s1  <= i_alpha * i_cos;
                p_bs_s1  <= i_beta  * i_sin;
                p_as_s1  <= i_alpha * i_sin;
                p_bc_s1  <= i_beta  * i_cos;
                valid_s1 <= i_valid;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 2: sum_d = ac+bs (=> i_d), sum_q = bc-as (=> i_q)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                sum_d_s2 <= (others => '0');
                sum_q_s2 <= (others => '0');
                valid_s2 <= '0';
            else
                sum_d_s2 <= resize(p_ac_s1, SUM_WIDTH) + resize(p_bs_s1, SUM_WIDTH);
                sum_q_s2 <= resize(p_bc_s1, SUM_WIDTH) - resize(p_as_s1, SUM_WIDTH);
                valid_s2 <= valid_s1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 3: round + saturate to Q1.15 output
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                o_d     <= (others => '0');
                o_q     <= (others => '0');
                o_valid <= '0';
            else
                o_d     <= round_sat(sum_d_s2);
                o_q     <= round_sat(sum_q_s2);
                o_valid <= valid_s2;
            end if;
        end if;
    end process;

end architecture rtl;
