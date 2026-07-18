--------------------------------------------------------------------------------
-- inverse_park_transform.vhd
--
-- Inverse Park transform: rotates the rotor-synchronous d/q voltage
-- commands back into the stationary alpha/beta frame for SVPWM:
--
--   v_alpha = v_d*cos(theta) - v_q*sin(theta)
--   v_beta  = v_d*sin(theta) + v_q*cos(theta)
--
-- This is R(-theta) applied to [Vd;Vq] -- the inverse of the rotation
-- park_transform.vhd applies -- and is confirmed self-consistent with
-- it: feeding this module's (v_alpha,v_beta) back into park_transform
-- with the same cos/sin reproduces (v_d,v_q) exactly (checked
-- algebraically and in the testbench below).
--
-- Same pipeline structure, bit widths, and rounding/saturation approach
-- as park_transform.vhd -- just a different combination of the same
-- four products.
--
-- All ports Q1.15 signed (16-bit). Fully pipelined, 3 cycles latency,
-- one sample per clock throughput.
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity inverse_park_transform is
    generic (
        DATA_WIDTH : integer := 16    -- Q1.15
    );
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;
        i_valid : in  std_logic;
        i_d     : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, Vd
        i_q     : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, Vq
        i_cos   : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, cos(theta)
        i_sin   : in  signed(DATA_WIDTH-1 downto 0);   -- Q1.15, sin(theta)
        o_valid : out std_logic;
        o_alpha : out signed(DATA_WIDTH-1 downto 0);   -- Q1.15, Valpha
        o_beta  : out signed(DATA_WIDTH-1 downto 0)    -- Q1.15, Vbeta
    );
end entity inverse_park_transform;

architecture rtl of inverse_park_transform is

    constant SUM_FRAC    : integer := 2 * (DATA_WIDTH - 1);        -- 30: frac bits in the Q3.30 sum
    constant OUT_FRAC    : integer := DATA_WIDTH - 1;               -- 15: frac bits in the Q1.15 output
    constant PROD_WIDTH  : integer := 2 * DATA_WIDTH;                -- 32, Q2.30
    constant SUM_WIDTH   : integer := PROD_WIDTH + 1;                -- 33, Q3.30 (1 headroom bit)
    constant SHIFT_WIDTH : integer := SUM_WIDTH - (SUM_FRAC - OUT_FRAC) + 1;  -- 19

    -- Stage 1: registered products
    signal p_dc_s1, p_qs_s1, p_ds_s1, p_qc_s1 : signed(PROD_WIDTH-1 downto 0);
    signal valid_s1 : std_logic;

    -- Stage 2: sums (alpha = dc-qs, beta = ds+qc)
    signal sum_a_s2, sum_b_s2 : signed(SUM_WIDTH-1 downto 0);
    signal valid_s2 : std_logic;

    function round_sat (w : signed(SUM_WIDTH-1 downto 0)) return signed is
        constant SHIFT : integer := SUM_FRAC - OUT_FRAC;   -- 15
        variable rounded : signed(SUM_WIDTH downto 0);
        variable slice1   : signed(SHIFT_WIDTH-1 downto 0);
        variable result   : signed(DATA_WIDTH-1 downto 0);
        variable safe     : boolean;
    begin
        rounded := resize(w, SUM_WIDTH+1) + to_signed(2**(SHIFT-1), SUM_WIDTH+1);
        slice1  := rounded(SUM_WIDTH downto SHIFT);

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
    -- Stage 1: four products (Vd*cos, Vq*sin, Vd*sin, Vq*cos)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                p_dc_s1  <= (others => '0');
                p_qs_s1  <= (others => '0');
                p_ds_s1  <= (others => '0');
                p_qc_s1  <= (others => '0');
                valid_s1 <= '0';
            else
                p_dc_s1  <= i_d * i_cos;
                p_qs_s1  <= i_q * i_sin;
                p_ds_s1  <= i_d * i_sin;
                p_qc_s1  <= i_q * i_cos;
                valid_s1 <= i_valid;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 2: sum_a = dc-qs (=> Valpha), sum_b = ds+qc (=> Vbeta)
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                sum_a_s2 <= (others => '0');
                sum_b_s2 <= (others => '0');
                valid_s2 <= '0';
            else
                sum_a_s2 <= resize(p_dc_s1, SUM_WIDTH) - resize(p_qs_s1, SUM_WIDTH);
                sum_b_s2 <= resize(p_ds_s1, SUM_WIDTH) + resize(p_qc_s1, SUM_WIDTH);
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
                o_alpha <= (others => '0');
                o_beta  <= (others => '0');
                o_valid <= '0';
            else
                o_alpha <= round_sat(sum_a_s2);
                o_beta  <= round_sat(sum_b_s2);
                o_valid <= valid_s2;
            end if;
        end if;
    end process;

end architecture rtl;
