--------------------------------------------------------------------------------
-- svpwm.vhd
--
-- Space Vector PWM via the min-max (third-harmonic / zero-sequence
-- injection) method: mathematically equivalent to textbook sector-based
-- SVPWM, but needs no sector lookup or angle at all -- just an inverse
-- Clarke, a min/max/offset, and an add. Verified equivalent to the
-- sector-based formulation offline (reconstructing Valpha/Vbeta from the
-- resulting duty cycles via the zero-sequence-rejecting Clarke transform
-- reproduces the original command to floating-point precision).
--
--   Va = Valpha
--   Vb = -Valpha/2 + (sqrt(3)/2)*Vbeta
--   Vc = -Valpha/2 - (sqrt(3)/2)*Vbeta
--   offset = -(max(Va,Vb,Vc) + min(Va,Vb,Vc)) / 2
--   Va',Vb',Vc' = Va+offset, Vb+offset, Vc+offset   (each now in [-0.5,0.5]
--                                                     for valid inputs)
--   duty = Vx' + 0.5   (implemented as a single MSB flip -- see below)
--
-- Valid input range: |Valpha,Vbeta| within the linear modulation region,
-- magnitude <= 1/sqrt(3) (the hexagon's inscribed circle). Output is
-- still saturated defensively in case an upstream stage overshoots that.
--
-- Converting the centered, signed Q1.15 value to an unsigned duty cycle
-- (0 = 0%, 65535 ~= 100%) is just "value + 32768 mod 65536" -- which for
-- a two's-complement number is exactly "flip the sign bit", no adder
-- needed.
--
-- Outputs a duty cycle per phase, not switching edges directly -- pairs
-- with a separate PWM-generator/counter-compare block (e.g. the one from
-- the earlier PID controller project) to produce actual gate signals.
--
-- Fully pipelined, 4 cycles latency, one sample per clock throughput.
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity svpwm is
    generic (
        DATA_WIDTH : integer := 16;   -- Q1.15
        WORK_WIDTH : integer := 19    -- Q4.15, headroom during inverse-Clarke/centering
    );
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic;
        i_valid  : in  std_logic;
        i_valpha : in  signed(DATA_WIDTH-1 downto 0);
        i_vbeta  : in  signed(DATA_WIDTH-1 downto 0);
        o_valid  : out std_logic;
        o_duty_a : out unsigned(DATA_WIDTH-1 downto 0);   -- Q0.16-equivalent, 0=0%, 65535~=100%
        o_duty_b : out unsigned(DATA_WIDTH-1 downto 0);
        o_duty_c : out unsigned(DATA_WIDTH-1 downto 0)
    );
end entity svpwm;

architecture rtl of svpwm is

    constant FRAC_BITS  : integer := DATA_WIDTH - 1;                 -- 15
    -- sqrt(3)/2 = 0.8660254037844387, Q1.15
    constant SQRT3_2    : signed(DATA_WIDTH-1 downto 0) := to_signed(28378, DATA_WIDTH);
    constant MUL_WIDTH  : integer := 2 * DATA_WIDTH;                 -- 32, Q2.30

    -- Stage 1: widen Va, compute Vbeta*sqrt(3)/2 (raw product)
    signal va_s1        : signed(WORK_WIDTH-1 downto 0);
    signal vbeta_mul_s1 : signed(MUL_WIDTH-1 downto 0);
    signal valid_s1     : std_logic;

    -- Stage 2: Vb, Vc via inverse Clarke
    signal va_s2, vb_s2, vc_s2 : signed(WORK_WIDTH-1 downto 0);
    signal valid_s2 : std_logic;

    -- Stage 3: offset = -(max+min)/2
    signal va_s3, vb_s3, vc_s3 : signed(WORK_WIDTH-1 downto 0);
    signal offset_s3 : signed(WORK_WIDTH-1 downto 0);
    signal valid_s3 : std_logic;

    -- Stage 4: centered + saturated to DATA_WIDTH, then MSB-flip to unsigned duty
    signal valid_s4 : std_logic;

    function round_shift15 (w : signed(MUL_WIDTH-1 downto 0)) return signed is
        variable rounded : signed(MUL_WIDTH downto 0);
    begin
        rounded := resize(w, MUL_WIDTH+1) + to_signed(2**(FRAC_BITS-1), MUL_WIDTH+1);
        return resize(rounded(MUL_WIDTH downto FRAC_BITS), WORK_WIDTH);
    end function;

    function saturate (w : signed(WORK_WIDTH-1 downto 0)) return signed is
        variable result : signed(DATA_WIDTH-1 downto 0);
        variable safe   : boolean;
    begin
        safe := true;
        for b in DATA_WIDTH-1 to WORK_WIDTH-2 loop
            if w(b+1) /= w(b) then
                safe := false;
            end if;
        end loop;
        if safe then
            result := w(DATA_WIDTH-1 downto 0);
        elsif w(WORK_WIDTH-1) = '1' then
            result := to_signed(-(2**(DATA_WIDTH-1)), DATA_WIDTH);
        else
            result := to_signed(2**(DATA_WIDTH-1)-1, DATA_WIDTH);
        end if;
        return result;
    end function;

begin

    ----------------------------------------------------------------------
    -- Stage 1
    ----------------------------------------------------------------------
    process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                va_s1 <= (others => '0');
                vbeta_mul_s1 <= (others => '0');
                valid_s1 <= '0';
            else
                va_s1 <= resize(i_valpha, WORK_WIDTH);
                vbeta_mul_s1 <= i_vbeta * SQRT3_2;
                valid_s1 <= i_valid;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 2: Vb = -Va/2 + Vbeta*sqrt3/2 ,  Vc = -Va/2 - Vbeta*sqrt3/2
    ----------------------------------------------------------------------
    process (clk)
        variable half_va : signed(WORK_WIDTH-1 downto 0);
        variable vbeta_scaled : signed(WORK_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                va_s2 <= (others => '0');
                vb_s2 <= (others => '0');
                vc_s2 <= (others => '0');
                valid_s2 <= '0';
            else
                half_va := shift_right(va_s1, 1);
                vbeta_scaled := round_shift15(vbeta_mul_s1);
                va_s2 <= va_s1;
                vb_s2 <= (-half_va) + vbeta_scaled;
                vc_s2 <= (-half_va) - vbeta_scaled;
                valid_s2 <= valid_s1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 3: offset = -(max(Va,Vb,Vc) + min(Va,Vb,Vc)) / 2
    ----------------------------------------------------------------------
    process (clk)
        variable vmax, vmin : signed(WORK_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                va_s3 <= (others => '0');
                vb_s3 <= (others => '0');
                vc_s3 <= (others => '0');
                offset_s3 <= (others => '0');
                valid_s3 <= '0';
            else
                vmax := va_s2;
                if vb_s2 > vmax then vmax := vb_s2; end if;
                if vc_s2 > vmax then vmax := vc_s2; end if;

                vmin := va_s2;
                if vb_s2 < vmin then vmin := vb_s2; end if;
                if vc_s2 < vmin then vmin := vc_s2; end if;

                va_s3 <= va_s2;
                vb_s3 <= vb_s2;
                vc_s3 <= vc_s2;
                offset_s3 <= shift_right(-(vmax + vmin), 1);
                valid_s3 <= valid_s2;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Stage 4: center, saturate to Q1.15, flip sign bit -> unsigned duty
    ----------------------------------------------------------------------
    process (clk)
        variable a_sat, b_sat, c_sat : signed(DATA_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                o_duty_a <= (others => '0');
                o_duty_b <= (others => '0');
                o_duty_c <= (others => '0');
                o_valid  <= '0';
            else
                a_sat := saturate(va_s3 + offset_s3);
                b_sat := saturate(vb_s3 + offset_s3);
                c_sat := saturate(vc_s3 + offset_s3);

                o_duty_a <= (not a_sat(DATA_WIDTH-1)) & unsigned(a_sat(DATA_WIDTH-2 downto 0));
                o_duty_b <= (not b_sat(DATA_WIDTH-1)) & unsigned(b_sat(DATA_WIDTH-2 downto 0));
                o_duty_c <= (not c_sat(DATA_WIDTH-1)) & unsigned(c_sat(DATA_WIDTH-2 downto 0));
                o_valid  <= valid_s3;
            end if;
        end if;
    end process;

end architecture rtl;
