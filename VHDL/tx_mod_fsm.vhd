library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tx_mod is
    Port (
        clk       : in  STD_LOGIC;  
        tx_data   : in  STD_LOGIC_VECTOR(7 downto 0);
        tx_out    : out STD_LOGIC
    );
end tx_mod;

architecture Behavioral of tx_mod is

    constant CLK_FREQ       : integer := 100_000_000;
    constant BAUD_RATE      : integer := 115200;
    constant BAUD_DIVISOR   : integer := CLK_FREQ / BAUD_RATE;
    constant TRANSMIT_PERIOD: integer := CLK_FREQ / 10; 

    type state_type is (IDLE, START, DATA, STOP);
    signal state         : state_type := IDLE;
    signal bit_index     : integer range 0 to 7 := 0;
    signal baud_counter  : integer range 0 to BAUD_DIVISOR - 1 := 0;
    signal tx_shift_reg  : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');
    signal tx_reg        : STD_LOGIC := '1'; 
    signal transmit_timer: integer range 0 to TRANSMIT_PERIOD - 1 := 0;
    signal transmit_trigger: STD_LOGIC := '0'; 
begin

    tx_out <= tx_reg;

process(clk)
begin
    if rising_edge(clk) then
      
        if transmit_timer = TRANSMIT_PERIOD - 1 then
            transmit_trigger <= '1';
            transmit_timer <= 0;
        else
            transmit_timer <= transmit_timer + 1;
            transmit_trigger <= '0';
        end if;

      
        case state is
            when IDLE =>
                
                if transmit_trigger = '1' then
                    tx_shift_reg <= tx_data; 
                    state <= START;
                end if;

            when START =>
           
                tx_reg <= '0'; -- Start bit
                if baud_counter = BAUD_DIVISOR - 1 then
                    baud_counter <= 0;
                    state <= DATA;
                    bit_index <= 0;
                else
                    baud_counter <= baud_counter + 1;
                end if;

            when DATA =>
          
                if baud_counter = BAUD_DIVISOR - 1 then
                    baud_counter <= 0;
                    if bit_index = 7 then
                        state <= STOP;
                        tx_reg <= '1'; 
                    else
                        tx_reg <= tx_shift_reg(bit_index);
                        bit_index <= bit_index + 1;
                    end if;
                else
                    baud_counter <= baud_counter + 1;
                end if;

            when STOP =>

                tx_reg <= '1';
                if baud_counter = BAUD_DIVISOR - 1 then
                    baud_counter <= 0;
                    state <= IDLE;
                else
                    baud_counter <= baud_counter + 1;
                end if;
        end case;
    end if;
end process;

end Behavioral;

