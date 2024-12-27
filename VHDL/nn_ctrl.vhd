


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity nn_ctrl is
    Port (  i_Clk : in STD_LOGIC;
            ap_ready : in STD_LOGIC;
            ap_start : out STD_LOGIC;
            ap_done  : in std_logic;
            ap_idle  : in std_logic;
            ap_rst   : out std_logic;
            rstb_busy: in std_logic;
            
            num_out : out std_logic_vector(31 downto 0);
            
            nn_res_in: in  std_logic_vector(31 downto 0)
           
           );
end nn_ctrl;

architecture Behavioral of nn_ctrl is

    signal start_signal :   std_logic := '0';
    signal data_out    :   std_logic_vector(31 downto 0);
    
    signal pred         :   integer := 0;
    
begin

    ------------------  Start NN  ------------------
    PROCESS(i_Clk, start_signal)
      VARIABLE cnt : INTEGER := 0;
    BEGIN
        if rising_edge(i_Clk) then
            if cnt < 5 then
                cnt := cnt + 1;
            end if;
        end if;
        
        if cnt > 2 then
            if ap_ready = '1' or ap_idle = '1' then
                if rstb_busy = '0' then
                    start_signal <= '1';
                else 
                    start_signal <= '0';
                end if;
            end if;
        end if;
    END PROCESS;


    pred <= to_integer(signed(nn_res_in));
        

    ap_start <= start_signal;

    ap_rst <= '1';
    
    num_out <= data_out;

    
end Behavioral;

