library IEEE;
use ieee.std_logic_1164.all ;

entity gh_clk_ce_div is 
    GENERIC (divide_ratio : natural :=8);
     port(
         CLK : in STD_LOGIC;
         rst : in STD_LOGIC;
         Q : out STD_LOGIC
         );
end gh_clk_ce_div;



architecture a of gh_clk_ce_div is  
--constant divide_ratio : natural := 5;

begin

process(CLK,rst)    
    VARIABLE count : INTEGER RANGE 0 TO divide_ratio;
begin -- 
    if (rst = '0') then
        count := 0;
        Q <= '0';
    elsif (rising_edge(CLK)) then
        if (count = (divide_ratio -1)) then 
            count := 0;
            Q <= '1';
        else
            count := count + 1;
            Q <= '0';
        end if;
    end if;
end process;

end a;