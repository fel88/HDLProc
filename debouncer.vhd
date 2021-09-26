library ieee;
use ieee.std_logic_1164.all;    
        use IEEE.std_logic_arith.all;
        
        use IEEE.numeric_std.all;
        
        use IEEE.std_logic_unsigned.all;
		  
entity debouncer is	
	port 
	(
		clk		: in std_logic;			
		inp		: in  std_logic;
		outp: out std_logic
		
	);

end entity;

architecture rtl of debouncer is


begin

process(clk)

variable v: std_logic;

begin
if rising_edge(clk)then
if(inp='0' and v='0') then

v:='1';
outp<='1';
end if;

if(inp='1' and v='1')then
v:='0';
outp<='0';
end if;

	
end if;

end process;


end rtl;
