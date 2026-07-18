--------------------------------------------------------------------------------
-- cordic_sincos.vhd
--
-- CORDIC (COordinate Rotation DIgital Computer), rotation mode, circular
-- coordinate system: computes cos(theta) and sin(theta) using only
-- shifts, adds/subtracts, and a small ROM of arctangent constants --
-- no multipliers.
--
-- Per-iteration update (i = 0 .. N-1):
--   d_i = +1 if z_i >= 0, else -1
--   x_{i+1} = x_i - d_i * y_i * 2^-i
--   y_{i+1} = y_i + d_i * x_i * 2^-i
--   z_{i+1} = z_i - d_i * atan(2^-i)
--
-- Standard CORDIC only converges for |theta| <~ 99.7 deg, so the input
-- is first range-reduced into [-pi/2, pi/2] using
-- cos(theta)=-cos(theta-+pi), sin(theta)=-sin(theta-+pi), with the sign
-- flip re-applied to the result afterwards.
--
-- x is pre-loaded with 1/K (K = CORDIC gain product of this many
-- iterations), which pre-compensates the algorithm's inherent magnitude
-- growth so no final multiply is needed.
--
-- NOT pipelined: one sin/cos computation takes ITERATIONS+2 clock
-- cycles; o_busy stays high for the duration. This trades throughput for
-- zero multiplier usage, per the CORDIC-vs-LUT trade-off discussed in
-- Part 1.
--
-- Formats:
--   i_theta      : Q4.18 signed, 22-bit, radians, expected in [-pi, pi)
--   internal x,y : Q1.19 signed, 20-bit
--   o_cos, o_sin : Q1.15 signed, 16-bit
--
-- Part of: FPGA-Based Field-Oriented Control for BLDC/PMSM Motors
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cordic_sincos is
    generic (
        ITERATIONS  : integer := 16;
        ANGLE_WIDTH : integer := 22;    -- Q4.18
        ANGLE_FRAC  : integer := 18;
        XY_WIDTH    : integer := 21;    -- Q2.19 (1 extra integer bit of headroom -- see round_sat)
        XY_FRAC     : integer := 19;
        OUT_WIDTH   : integer := 16     -- Q1.15
    );
    port (
        clk     : in  std_logic;
        rst_n   : in  std_logic;
        i_valid : in  std_logic;
        i_theta : in  signed(ANGLE_WIDTH-1 downto 0);   -- Q4.18 radians
        o_busy  : out std_logic;
        o_valid : out std_logic;
        o_cos   : out signed(OUT_WIDTH-1 downto 0);      -- Q1.15
        o_sin   : out signed(OUT_WIDTH-1 downto 0)       -- Q1.15
    );
end entity cordic_sincos;

architecture rtl of cordic_sincos is

    type atan_table_t is array (0 to ITERATIONS-1) of signed(ANGLE_WIDTH-1 downto 0);
    constant ATAN_TABLE : atan_table_t := (
        to_signed(205887, ANGLE_WIDTH), to_signed(121542, ANGLE_WIDTH),
        to_signed(64220,  ANGLE_WIDTH), to_signed(32599,  ANGLE_WIDTH),
        to_signed(16363,  ANGLE_WIDTH), to_signed(8189,   ANGLE_WIDTH),
        to_signed(4096,   ANGLE_WIDTH), to_signed(2048,   ANGLE_WIDTH),
        to_signed(1024,   ANGLE_WIDTH), to_signed(512,    ANGLE_WIDTH),
        to_signed(256,    ANGLE_WIDTH), to_signed(128,    ANGLE_WIDTH),
        to_signed(64,     ANGLE_WIDTH), to_signed(32,     ANGLE_WIDTH),
        to_signed(16,     ANGLE_WIDTH), to_signed(8,      ANGLE_WIDTH)
    );

    -- 1/K = 0.6072529351031394 (K = CORDIC gain for 16 iterations), Q1.19
    constant INV_K  : signed(XY_WIDTH-1 downto 0)    := to_signed(318375, XY_WIDTH);
    constant PI_Q   : signed(ANGLE_WIDTH-1 downto 0) := to_signed(823550, ANGLE_WIDTH);  -- pi,   Q4.18
    constant PI_2_Q : signed(ANGLE_WIDTH-1 downto 0) := to_signed(411775, ANGLE_WIDTH);  -- pi/2, Q4.18

    type state_t is (IDLE, ITERATE, FINISH);
    signal state    : state_t := IDLE;
    signal x_reg, y_reg : signed(XY_WIDTH-1 downto 0);
    signal z_reg    : signed(ANGLE_WIDTH-1 downto 0);
    signal iter_cnt : integer range 0 to ITERATIONS-1;
    signal negate_r : std_logic;

    -- Round-to-nearest and saturate a Q2.(XY_FRAC) value to Q1.(OUT_WIDTH-1).
    -- x/y carry 1 extra integer bit of headroom beyond the [-1,1) output
    -- range, because values converging to exactly +-1.0 (e.g. theta=0,
    -- 90 deg) transiently need it -- see design notes.
    function round_sat (w : signed(XY_WIDTH-1 downto 0)) return signed is
        constant SHIFT       : integer := XY_FRAC - (OUT_WIDTH - 1);  -- fractional bits to drop
        constant SLICE_WIDTH : integer := XY_WIDTH - SHIFT + 1;
        variable rounded : signed(XY_WIDTH downto 0);
        variable slice1  : signed(SLICE_WIDTH-1 downto 0);
        variable result  : signed(OUT_WIDTH-1 downto 0);
        variable safe    : boolean;
    begin
        rounded := resize(w, XY_WIDTH+1) + to_signed(2**(SHIFT-1), XY_WIDTH+1);
        slice1  := rounded(XY_WIDTH downto SHIFT);

        safe := true;
        for b in OUT_WIDTH-1 to SLICE_WIDTH-2 loop
            if slice1(b+1) /= slice1(b) then
                safe := false;
            end if;
        end loop;

        if safe then
            result := slice1(OUT_WIDTH-1 downto 0);
        elsif slice1(SLICE_WIDTH-1) = '1' then
            result := to_signed(-(2**(OUT_WIDTH-1)), OUT_WIDTH);
        else
            result := to_signed(2**(OUT_WIDTH-1)-1, OUT_WIDTH);
        end if;
        return result;
    end function;

begin

    o_busy <= '1' when state /= IDLE else '0';

    process (clk)
        variable theta_f       : signed(ANGLE_WIDTH-1 downto 0);
        variable neg_f         : std_logic;
        variable dx, dy        : signed(XY_WIDTH-1 downto 0);
        variable dz            : signed(ANGLE_WIDTH-1 downto 0);
        variable x_shr, y_shr  : signed(XY_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                state    <= IDLE;
                x_reg    <= (others => '0');
                y_reg    <= (others => '0');
                z_reg    <= (others => '0');
                iter_cnt <= 0;
                negate_r <= '0';
                o_valid  <= '0';
                o_cos    <= (others => '0');
                o_sin    <= (others => '0');
            else
                o_valid <= '0';   -- default; pulsed high for one cycle in FINISH

                case state is

                    when IDLE =>
                        if i_valid = '1' then
                            if i_theta > PI_2_Q then
                                theta_f := i_theta - PI_Q;
                                neg_f   := '1';
                            elsif i_theta < -PI_2_Q then
                                theta_f := i_theta + PI_Q;
                                neg_f   := '1';
                            else
                                theta_f := i_theta;
                                neg_f   := '0';
                            end if;

                            x_reg    <= INV_K;
                            y_reg    <= (others => '0');
                            z_reg    <= theta_f;
                            negate_r <= neg_f;
                            iter_cnt <= 0;
                            state    <= ITERATE;
                        end if;

                    when ITERATE =>
                        x_shr := shift_right(x_reg, iter_cnt);
                        y_shr := shift_right(y_reg, iter_cnt);

                        if z_reg(ANGLE_WIDTH-1) = '0' then     -- z >= 0
                            dx := x_reg - y_shr;
                            dy := y_reg + x_shr;
                            dz := z_reg - ATAN_TABLE(iter_cnt);
                        else                                     -- z < 0
                            dx := x_reg + y_shr;
                            dy := y_reg - x_shr;
                            dz := z_reg + ATAN_TABLE(iter_cnt);
                        end if;

                        x_reg <= dx;
                        y_reg <= dy;
                        z_reg <= dz;

                        if iter_cnt = ITERATIONS-1 then
                            state <= FINISH;
                        else
                            iter_cnt <= iter_cnt + 1;
                        end if;

                    when FINISH =>
                        if negate_r = '1' then
                            o_cos <= round_sat(-x_reg);
                            o_sin <= round_sat(-y_reg);
                        else
                            o_cos <= round_sat(x_reg);
                            o_sin <= round_sat(y_reg);
                        end if;
                        o_valid <= '1';
                        state   <= IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
