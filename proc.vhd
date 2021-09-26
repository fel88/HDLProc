library ieee;
use ieee.std_logic_1164.all;    
        use IEEE.std_logic_arith.all;        
        use IEEE.numeric_std.all;        
        use IEEE.std_logic_unsigned.all;
		  
entity decoder is	
	port 
	(
		clk		: in std_logic;		
		in_sd_clk		: in std_logic;		
	
		
		
		reset		: in  std_logic;
		cmd: in std_logic_vector(3 downto 0);
		cmd_ready : in std_logic;
		cmd_wait : out std_logic;
		
		outp: out std_logic_vector(3 downto 0);
		
		outpA: out std_logic_vector(31 downto 0);
		outpB: out std_logic_vector(31 downto 0);
		outpC: out std_logic_vector(31 downto 0);
		outpD: out std_logic_vector(31 downto 0);
		outpE: out std_logic_vector(31 downto 0);
		outpF: out std_logic_vector(31 downto 0);
		
		inpA: in std_logic_vector(31 downto 0);
		inpB: in std_logic_vector(31 downto 0);
		inpC: in std_logic_vector(31 downto 0);
		
	--	inpA: in std_logic_vector(31 downto 0);
	--	inpB: in std_logic_vector(31 downto 0);
	--	inpC: in std_logic_vector(31 downto 0);
		
		outflag:out std_logic;
		
		---ext RAM
		
		e_ram_addr	: out  natural range 0 to 2**12 - 1;
		e_ram_data	:  out std_logic_vector(31 downto 0);
		--e_ram_q	:  in std_logic_vector(31 downto 0);
		e_ram_we		:  out std_logic := '1';

		-- end ext ram
		--sdram
		sdram_s1_address: out STD_LOGIC_VECTOR (21 DOWNTO 0);
		
		
		sdram_s1_writedata: out STD_LOGIC_VECTOR (15 DOWNTO 0);
		sdram_s1_read_n: out STD_LOGIC;
		sdram_s1_write_n: out STD_LOGIC;
		sdram_s1_readdata: in STD_LOGIC_VECTOR (15 DOWNTO 0);
		sdram_s1_readdatavalid: in STD_LOGIC;
		sdram_s1_waitrequest     : in STD_LOGIC;
		--end sdram
		
		--external fifo read interface
		fifo_rd : out std_logic;
		fifo_data: in std_logic_vector(31 downto 0);
		fifo_empty : in std_logic;
		--end fifo
		
		-- external SD CARD interface
	SD_DAT0 		: inout std_logic;		
	SD_DAT3		: inout std_logic;		
	SD_CMD 		: inout std_logic;		
	SD_CLK 		: out std_logic;		
	SD_WP_N		: in std_logic			
	);

end entity;

architecture rtl of decoder is
signal s_ram_selector:std_logic_vector(1 downto 0):="00";
signal s_rd : std_logic;
signal s_read_flag : std_logic;
signal s_write_flag : std_logic;
signal	 s_wr : std_logic;
signal	 s_dm_in : std_logic;	-- data mode, 0 = write continuously, 1 = write single block

signal	 s_din :  std_logic_vector(7 downto 0);
signal	 s_dout :  std_logic_vector(7 downto 0);
signal	 s_dout_clk :  std_logic;
signal	 s_address :  std_logic_vector(31 downto 0);
--MBR registers
signal FirstSectionInVolume1: std_logic_vector(31 downto 0);
signal Partition_Type :std_logic_vector(7 downto 0);
signal REG1 :std_logic_vector(7 downto 0);

signal NumFATs :std_logic_vector(7 downto 0);
signal FATSize:std_logic_vector(15 downto 0);
signal RsvdSecCnt :std_logic_vector(15 downto 0);
signal RootDirectoryEntrySecter	:	std_logic_vector(31 downto 0);
signal NUM_FILES_IN_ROOT_DIRECTORY	:	natural range 0 to 127;
--end MBR registers

--Ram signals
	
signal		s_ram_addr	:  natural range 0 to 2**9 - 1;
signal		s_ram_addr_1	:  natural range 0 to 2**9 - 1;
signal		s_ram_addr_2	:  natural range 0 to 2**9 - 1;

signal		s_varram_addr	:  natural range 0 to 2**12 - 1;
signal		s_varram_addr_1	:  natural range 0 to 2**12 - 1;
signal		s_varram_addr_2	:  natural range 0 to 2**12 - 1;

signal s_ram_data	: std_logic_vector(7 downto 0);
signal s_varram_data	: std_logic_vector(31 downto 0);
signal s_varram_data_1	: std_logic_vector(31 downto 0);
signal s_varram_data_2	: std_logic_vector(31 downto 0);

signal s_ram_we		:  std_logic := '1';
signal s_varram_we		:  std_logic := '1';
signal s_varram_we_1		:  std_logic := '1';
signal s_varram_we_2		:  std_logic := '1';
signal s_ram_we_1		:  std_logic := '1';
signal s_ram_we_2		:  std_logic := '1';

signal s_ram_q		:  std_logic_vector(7 downto 0);
signal s_varram_q		:  std_logic_vector(31 downto 0);
signal s_varram_q_1		:  std_logic_vector(31 downto 0);
--signal s_varram_q_2		:  std_logic_vector(31 downto 0);

signal		s_rom_addr	:  natural range 0 to 2**12 - 1;
signal s_rom_q		:  std_logic_vector(15 downto 0);
signal s_progrom_rden		:  std_logic;

signal		s_ram_addr_b	:  natural range 0 to 2**9 - 1;
signal s_ram_data_b	: std_logic_vector(7 downto 0);
signal s_ram_we_b		:  std_logic := '1';
signal s_ram_q_b		:  std_logic_vector(7 downto 0);
--end ram signals

signal tt1 : std_logic_vector(3 downto 0);




type states is (
	RST,
	STATE4,
--	STATE1,
--	STATE2,
--	STATE3,
	READ_SECTOR,
	READ_SECTOR_WAIT,
	FINISH_READ_SECTOR,
	WRITE_SECTOR,	
	WRITE_SECTOR_WAIT,
	FINISH_WRITE_SECTOR,
	RELAX,
	READ_REGISTER,
	READ_VARIABLE,	
	WRITE_VARIABLE,
	INST_FETCH,	
	FETCH_WAIT,		
	INST_EXECUTE,
	READ_VARIABLE_SDRAM,
	WRITE_VARIABLE_SDRAM,
	SDRAM_WAIT,
	SDRAM_WAIT2,
	SDRAM_WAIT3,
	SDRAM_WAIT4,
	FPU_INT2FLOAT,
	FPU_FLOAT2INT,
	FPU_MUL,
	FPU_MUL_B,
	FPU_DIV,
	FPU_ADD,
	FPU_ADD_B,
	FPU_INT2FLOAT_WAIT,
	FPU_FLOAT2INT_WAIT,
	FPU_MUL_WAIT,
	FPU_DIV_WAIT,
	FPU_ADD_WAIT,
	FIFO_READ_WAIT,
	FIFO_READ,
	SFIFO_WRITE,--stack
	SFIFO_READ--stack
);

signal state, return_state : states;
signal latch_cmd_ready:std_logic;
signal latch2_cmd_ready:std_logic;

 signal s_adder_input_a: std_logic_vector(31 downto 0);
        signal  s_adder_input_b: std_logic_vector(31 downto 0);
        signal  s_adder_input_a_stb: std_logic;
        signal  s_adder_input_b_stb: std_logic;
        signal  s_adder_output_z_ack: std_logic;
        
        signal  s_adder_output_z: std_logic_vector(31 downto 0);
        signal s_adder_output_z_stb: std_logic;
        signal s_adder_input_a_ack: std_logic;
        signal s_adder_input_b_ack: std_logic;

			signal s_f2i_input_a: std_logic_vector(31 downto 0);        
        signal s_f2i_input_a_stb: std_logic;        
        signal s_f2i_output_z_ack: std_logic;        
        signal s_f2i_output_z: std_logic_vector(31 downto 0);
        signal s_f2i_output_z_stb: std_logic;
        signal s_f2i_input_a_ack: std_logic;
		  
		  signal s_i2f_input_a: std_logic_vector(31 downto 0);        
        signal s_i2f_input_a_stb: std_logic;        
        signal s_i2f_output_z_ack: std_logic;        
        signal s_i2f_output_z: std_logic_vector(31 downto 0);
        signal s_i2f_output_z_stb: std_logic;
        signal s_i2f_input_a_ack: std_logic;
		  
		  
		    signal s_mul_input_a: std_logic_vector(31 downto 0);
        signal s_mul_input_b: std_logic_vector(31 downto 0);
        signal s_mul_input_a_stb: std_logic;
        signal s_mul_input_b_stb: std_logic;
        signal s_mul_output_z_ack: std_logic;
        
        signal s_mul_output_z: std_logic_vector(31 downto 0);
        signal s_mul_output_z_stb: std_logic;
        signal s_mul_input_a_ack: std_logic;
        signal s_mul_input_b_ack: std_logic;
		  

		  component stack_fifo IS
	PORT
	(
		clock		: IN STD_LOGIC ;
		data		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		rdreq		: IN STD_LOGIC ;
		wrreq		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END component;

component adder is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;


component multiplier is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;

component divider is
port (
        input_a:in std_logic_vector(31 downto 0);
        input_b:in std_logic_vector(31 downto 0);
        input_a_stb:in std_logic;
        input_b_stb:in std_logic;
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic;
        input_b_ack:out std_logic
		  
  );
end component;

component int_to_float is
port (
        input_a:in std_logic_vector(31 downto 0);
        
        input_a_stb:in std_logic;
        
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic
        
		  
  );
end component;

component float_to_int is
port (
        input_a:in std_logic_vector(31 downto 0);       
        input_a_stb:in std_logic;        
        output_z_ack:in std_logic;
        clk:in std_logic;
        rst:in std_logic;
        output_z:out std_logic_vector(31 downto 0);
        output_z_stb:out std_logic;
        input_a_ack:out std_logic        
		  
  );
end component;

	component ram1p IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (8 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (7 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
	);
end component ram1p;


component varram IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (9 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
END component;

component progrom IS
	PORT
	(
	address		: IN STD_LOGIC_VECTOR (11 DOWNTO 0);
		clock		: IN STD_LOGIC  := '1';
--		rden		: IN STD_LOGIC  := '1';
		q		: OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
		
		
	);
end component progrom;

component sd_controller is
port (
	--sd card spi signals
	cs : out std_logic;
	mosi : out std_logic;
	miso : in std_logic;
	sclk : out std_logic;
	-- end sd card spi signals
	
	--output ram write signals		
		ram_addr	: out natural range 0 to 2**9 - 1;
		ram_data	: out std_logic_vector((8-1) downto 0);
		ram_we		: out std_logic ;
	--input ram write signals		
		--iram_addr	: in natural range 0 to 2**9 - 1;
		iram_data	: in std_logic_vector((8-1) downto 0);		
		
	--end output ram write signals
	
	addr : in std_logic_vector(31 downto 0);
	rd : in std_logic;
	wr : in std_logic;
	dm_in : in std_logic;	-- data mode, 0 = write continuously, 1 = write single block
	reset : in std_logic;
	din : in std_logic_vector(7 downto 0);
	dout : out std_logic_vector(7 downto 0);	
	read_flag: out std_logic;
	write_flag: out std_logic;
	clk : in std_logic	-- twice the SPI clk
);
end component;

signal counter: integer;
	
	signal	s_sf_data		:  STD_LOGIC_VECTOR (31 DOWNTO 0);
		signal s_sf_rd		:  STD_LOGIC ;
		signal s_sf_wr		: STD_LOGIC ;
		signal s_sf_q		:  STD_LOGIC_VECTOR (31 DOWNTO 0);

signal regs1:std_logic_vector(31 downto 0);
signal regs2:std_logic_vector(31 downto 0);
signal regs3:std_logic_vector(31 downto 0);


begin

stack0: stack_fifo 
	PORT map
	(
		clock		=>clk,
		data		=>s_sf_data,
		rdreq		=>s_sf_rd,
		wrreq		=>s_sf_wr,
		q		=>s_sf_q
	);



--fpu
fpu0: adder 
port map(
        input_a=>s_adder_input_a,
        input_b=>s_adder_input_b,
        input_a_stb=>s_adder_input_a_stb,
        input_b_stb=>s_adder_input_b_stb,
        output_z_ack=>s_adder_output_z_ack,
        clk=>clk,
        rst=>reset,
        output_z=>s_adder_output_z,
        output_z_stb=>s_adder_output_z_stb,
        input_a_ack=>s_adder_input_a_ack,
        input_b_ack=>s_adder_input_b_ack
		  
  );
fpu1: int_to_float 
port map(
        input_a=>s_i2f_input_a,        
        input_a_stb=>s_i2f_input_a_stb,        
        output_z_ack=>s_i2f_output_z_ack,
        clk=>clk,
        rst=>reset,
        output_z=>s_i2f_output_z,
        output_z_stb=>s_i2f_output_z_stb
        --input_a_ack:out std_logic;
        --input_b_ack:out std_logic
		  
  );
  
  fpu2: multiplier
port map(
			input_a=>s_mul_input_a,
        input_b=>s_mul_input_b,
        input_a_stb=>s_mul_input_a_stb,
        input_b_stb=>s_mul_input_b_stb,
        output_z_ack=>s_mul_output_z_ack,
        clk=>clk,
        rst=>reset,
        output_z=>s_mul_output_z,
        output_z_stb=>s_mul_output_z_stb,
        input_a_ack=>s_mul_input_a_ack,
        input_b_ack=>s_mul_input_b_ack
		  
  );
  
  fpu3: float_to_int 
port map(
        input_a=>s_f2i_input_a,        
        input_a_stb=>s_f2i_input_a_stb,        
        output_z_ack=>s_f2i_output_z_ack,
        clk=>clk,
        rst=>reset,
        output_z=>s_f2i_output_z,
        output_z_stb=>s_f2i_output_z_stb        
		  
  );
--end fpu


sd0:sd_controller
port map
(
		cs=>SD_DAT3,
		mosi=>SD_CMD,
		sclk=>SD_CLK,
		miso=>SD_DAT0,
		rd=>s_rd,
		wr=>s_wr,
		dm_in=>s_dm_in,
		reset=>reset,
		din=>s_din,
		read_flag=>s_read_flag,
		write_flag=>s_write_flag,
		dout=>s_dout,
		clk=>in_sd_clk,
		addr=>s_address,		
		
		iram_data=>s_ram_q,
		

		ram_addr=>s_ram_addr_1,
		ram_we=>s_ram_we_1,
		ram_data=>s_ram_data

);

s_dm_in<='1';

process(clk)
variable v:integer range 0 to 16:=0;
begin
if reset='1' then
	v:=0;
elsif rising_Edge(clk) then

if(cmd_ready='1' and v=0) then
	latch_cmd_ready<='1';
	v:=5;
end if;
if v>2 then
	v:=v-1;
end if;
if v=2 then
	latch_cmd_ready<='0';
end if;
if cmd_ready='0' then
	v:=0;
end if;
end if;
end process;

process(clk)
variable v:integer range 0 to 100:=0;
begin
if reset='1' then
	v:=0;
elsif rising_Edge(clk) then

if(cmd_ready='1' and v=0) then
	latch2_cmd_ready<='1';
	v:=99;
end if;
if v>2 then
	v:=v-1;
end if;
if v=2 then
	latch2_cmd_ready<='0';
end if;
if cmd_ready='0' then
	v:=0;
end if;
end if;
end process;

cmd_wait<='1' when state=RST;-- others '0';
	s_varram_addr_1<=s_varram_addr	when s_ram_selector=x"01" else 0;	
	s_varram_addr_2<=s_varram_addr	when s_ram_selector=x"00" else 0;	
	s_varram_data_1<=s_varram_data when s_ram_selector=x"01" else x"00000000";	
	s_varram_data_2<=s_varram_data when s_ram_selector=x"00" else x"00000000";	
	s_varram_we_1<=s_varram_we when s_ram_selector=x"01" else '0';	
	s_varram_we_2<=s_varram_we when s_ram_selector=x"00" else '0'	;
	s_varram_q<=s_varram_q_1 ;
	--s_varram_q<=s_varram_q_1 when s_ram_selector=x"01" else s_varram_q_2;	
	--s_varram_q_1<=s_varram_q when s_ram_selector=x"01" else x"00000000";	
	--s_varram_q_2<=s_varram_q when s_ram_selector=x"00" else x"00000000";	
	
		e_ram_data<=s_varram_data_2;
			e_ram_addr<=s_varram_addr_2;
			e_ram_we<=s_varram_we_2;
			
process(clk)
variable vOutp:std_logic_vector(3 downto 0);
variable delay:integer range 0 to 50e6;
variable vAddress: std_logic_vector(31 downto 0);
variable aCounter: integer;
variable vReg: integer range 0 to 3;
variable vRegReadAddr: integer range 0 to 2**12-1;
variable vRegReadCount: integer range 0 to 4;
variable v3: integer range 0 to 10;
variable v4: integer range 0 to 10;
variable reg1: std_logic_Vector(31 downto 0);
variable reg2: std_logic_Vector(31 downto 0);
variable reg3: std_logic_Vector(31 downto 0);
variable reg4: std_logic_Vector(31 downto 0);
--variable div1: natural;
--variable div2: natural;
variable accum: std_logic_Vector(31 downto 0);--accumulator register
variable code_pointer: integer range 0 to 2**12-1;
variable stack_pointer: integer range 0 to 2**5-1;
--flags
variable z: std_logic;
variable s: std_logic;--sign 
variable o: std_logic;--overflow
variable cntr3: integer range 0 to 10;
variable timer: std_logic_Vector(31 downto 0);
variable timer_cntr: integer range 0 to 100000;--100Mhz=100000
variable instrx: std_logic_vector(15 downto 0);
begin

if( reset='1') then
	
	aCounter:=0;
	state<=RST;
	vOutp:="0000";
	--vAddress:=x"00010E00";
	vAddress:=x"00000000";
	code_pointer:=0;
	v3:=0;
	v4:=0;
	accum:=x"00000000";
	reg1:=x"00000000";
	reg2:=x"00000000";
	reg3:=x"00000000";
	reg4:=x"00000000";
	z:='0';
	stack_pointer:=0;
	timer_cntr:=0;
	timer:=x"00000000";
	outpA<=(others=>'0');
	outpB<=(others=>'0');
	outpC<=(others=>'0');
	
elsif rising_edge(clk) then
		
		timer_cntr:=timer_cntr+1;
		if(timer_cntr=100000)then
			timer_cntr:=0;
		end if;
		if(timer_cntr=1)then
			timer:=timer+1;
		end if;
		
		

		
case state is
	--------------------------------------------				
				when RST =>
				
										--ram start init
										s_ram_selector<="01";
										--end ram srart init
										
										
					s_rom_addr<=0;					
					vOutp:="0000";
					code_pointer:=0;
					if(latch_cmd_ready='1') then
						if(cmd="0001") then
							vAddress:=x"00060000";
							state<=READ_SECTOR;
							delay:=7;
							return_state<=RELAX;
						elsif(cmd="0010") then
							state<=READ_REGISTER;
							vRegReadCount:=3;
							vRegReadAddr:=454;
							delay:=7;
							v4:=0;
							return_state<=RELAX;
						elsif(cmd="0100") then
							state<=INST_FETCH;
						elsif(cmd="0011") then
							stack_pointer:=stack_pointer-1;
							vRegReadAddr:=stack_pointer+32;								
								
							state<=READ_VARIABLE;														
							
							delay:=7;
							v4:=0;
							return_state<=RELAX;
						elsif(cmd="0111") then
							vRegReadAddr:=stack_pointer+32;
							stack_pointer:=stack_pointer+1;		
														
							state<=WRITE_VARIABLE;														
							reg1:=reg1+1;							
							delay:=7;	
							v4:=0;							
							return_state<=RELAX;
						else
							--state<=STATE1;
						end if;
					end if;
	--------------------------------------------				
				when INST_FETCH=>
					--s_rom_addr<=code_pointer;					
					
					--delay:=3;
					delay:=2;
					v3:=0;
					--state<=FETCH_WAIT;
					state<=FETCH_WAIT;
					--return_state<=FETCH_WAIT;					
					--state<=INST_EXECUTE;					
	--------------------------------------------	
				
				when FETCH_WAIT=>			
					s_rom_addr<=code_pointer;		
					instrx:=s_rom_q;
					state<=INST_EXECUTE;
					code_pointer:=code_pointer+1;
					
				when INST_EXECUTE=>
					
			--	if v3=1 then
				case instrx(15 downto 8) is
				  when x"A0" =>  --shift and write to address register
						vAddress:=vAddress(23 downto 0)&instrx(7 downto 0);
					when x"D8" => --FPU commands
							case instrx(7 downto 0) is
								when x"01" => -- int2float										
										state<=FPU_INT2FLOAT;								
								when x"02" => --float adder								
										state<=FPU_ADD;								
								when x"03" => --float mul
										state<=FPU_MUL;								
								when x"04" => --float div
										state<=FPU_DIV;								
								when x"05" => --float to int
										state<=FPU_FLOAT2INT;	
								when others =>
							end case;
							--return_state<=INST_EXECUTE;	
							
					when x"A1" => --execute command 
								case instrx(7 downto 0) is
								 when x"01" => -- read sector
											state<=READ_SECTOR;								
								when x"02" =>  --read register
											reg1:=x"00000000";
											state<=READ_REGISTER;
											v4:=0;
									when x"03" => --read variable		
											s_ram_selector<="01";	
											state<=READ_VARIABLE;
											s_varram_we<='0';
											v4:=0;
									when x"13" => --FUSED: read variable + addres update		
											s_ram_selector<="01";	
											vRegReadAddr:=conv_integer(reg1);														
											state<=READ_VARIABLE;
											s_varram_we<='0';
											v4:=0;
									when x"04" => --write variable	
											s_ram_selector<="01";							
											state<=WRITE_VARIABLE;
											v4:=0;
									when x"14" => --write variable bus2	
											s_ram_selector<="00";
											state<=WRITE_VARIABLE;											
											v4:=0;
									when x"05" => --push reg1 to stack
											--todo: stack as FIFO? hard to debug..
											vRegReadAddr:=stack_pointer+32;
											stack_pointer:=stack_pointer+1;
											state<=WRITE_VARIABLE;
											v4:=0;
									when x"06"=>  --pop to reg1 from stack
											s_ram_selector<="01";	
											stack_pointer:=stack_pointer-1;
											vRegReadAddr:=stack_pointer+32;								
											state<=READ_VARIABLE;
											s_varram_we<='0';
											v4:=0;
									when x"07"=>  --write variable	 in sdram							
											state<=WRITE_VARIABLE_SDRAM;
											v4:=0;
											cntr3:=0;
									when x"08"=>  --read variable	 sdram							
											state<=READ_VARIABLE_SDRAM;
											v4:=0;
									when x"09"=>  --sd card write sector
											state<=WRITE_SECTOR;		
									when x"0A"=>  --fifo read
											state<=FIFO_READ_WAIT;	
									when x"0B"=>  -- stack read
											v4:=0;
											state<=SFIFO_READ;
									when x"0C"=>  -- stack write
											v4:=0;
											state<=SFIFO_WRITE;
										when others=> null;
									end case;									
									--return_state<=INST_EXECUTE;								
										
					when x"51" => -- A2 or A3 command set reg read addr
									vRegReadAddr:=conv_integer(instrx(8 downto 0));		
					when x"A4" => --copy reg1 to vAddress register
									vAddress:=reg1;		
									
					when x"A5" => --shift vAddress by 9 bit left (multiply *9)
								vAddress:=vAddress(22 downto 0)&"000000000";		
								
					when x"A6"=> --reset accumulator register
								accum:=x"00000000";		
							
					when x"A7" => --add reg1 to accumulator
							if(instrx(7 downto 4)=x"0") then
								accum:=accum+reg1;	
							else 
								accum:=accum-reg1;	
							end if;
								
					when x"A8" => --copy accumulator to address regidter
								vAddress:=accum;			
								
					when x"A9"=> -- set vRegReadCount register. num bytes to read register command
								vRegReadCount:=conv_integer(instrx(7 downto 0))-1;	
								
					when x"AA" =>--  cross register mov opertaions (reduced)
							
							case instrx(7 downto 4) is
								when x"0"=>
									reg2:=reg1;																						
								when x"1"=>
									reg1:=reg2;								
								when x"2"=>
									reg3:=reg1;	
								when x"3"=>
									reg1:=reg3;	
								when others => 
							end case;
							
--							
									
						when x"FC"=>
							if(instrx(7 downto 4)=x"1") then --timer store
								reg1:=timer;
							elsif(instrx(7 downto 4)=x"2") then --reset timer 
								timer:=x"00000000";
							end if;
					
								
						when x"AB" => -- set reg1=reg1*reg2
								----reg3:=conv_std_logic_vector(conv_integer(reg1*reg2),32);	
								--reg3:=ext(reg1*reg2,32);
								--reg1:=reg3;
								case instrx(7 downto 0) is
									when x"00" =>   
										--reg1:=ext(reg1*reg2,32);
										reg1:=std_logic_vector(to_unsigned(conv_integer(reg1)*conv_integer(reg2),reg1'length));
									when others =>		
										--div1:=conv_integer(reg1);
										--div2:=conv_integer(reg2);
										--div1:=div1/div2;
										reg1:=std_logic_vector(to_unsigned(conv_integer(reg1)/conv_integer(reg2),reg1'length));
								end case;
								
						
						
						when x"AC" => --shift and write to reg1
								reg1:=reg1(23 downto 0)&instrx(7 downto 0);
								
						when x"1C"=>  --store reg1 lower 8 bit 
								reg1:="000000000000000000000000"&instrx(7 downto 0);
						--elsif(s_rom_q(15 downto 8)=x"1B") then --store regAddr lower 8 bit 
							
								--vregReadAddr:=to_integer(unsigned(std_logic_vector(to_unsigned(vRegReadAddr, reg1'length))(23 downto 0)&s_rom_q(7 downto 0)));
						--!!!TRY TO DO!!!when x"2C"=>  --store reg1 lower 8 bit 
								--reg1:="000000000000000000000000"&s_rom_q(7 downto 0);		
								--vRegReadAddr:=conv_integer("000000000000000000000000"&s_rom_q(7 downto 0));	
						when x"AD"=>-- cmp (subtract) reg1=reg1-reg2
							case instrx(7 downto 0) is
								when x"00" =>   
									reg1:=reg1-reg2;
									if(reg1=x"00000000") then
										z:='1';									
										s:='0';
									else
										z:='0';
										if(reg1>reg2) then
											s:='1';
										else
											s:='0';
										end if;
									end if;
								when x"01" =>   
									if(z='0' and s='1') then
										code_pointer:=conv_integer(reg1);							
									end if;
								when x"02" =>  
										if(z='1' or s='1') then
											code_pointer:=conv_integer(reg1);							
										end if;	
								when x"03" =>   
										if(z='0' and  s='0') then
											code_pointer:=conv_integer(reg1);							
										end if;	
								when x"04" =>  
									if(z='1' or s='0') then
										code_pointer:=conv_integer(reg1);							
									end if;	
								when x"05" =>  
									reg1:="0000000000000000000000000000000"&z;
								when x"06" =>  
									reg1:="0000000000000000000000000000000"&s;
								when x"07" =>   
									reg1:=std_logic_vector(to_unsigned(code_pointer, reg1'length));
								when others =>--ad99
										
									if(reg1=reg2) then
										z:='1';									
										s:='0';
									else
										z:='0';
										if(reg1>reg2) then
											s:='1';
										else
											s:='0';
										end if;
									end if;
							end case;


						when x"3D"=>	 -- bitwise ops
								case instrx(7 downto 4) is
									when x"A"=> -- bitwise AND
											reg1:=reg1 AND reg2;		
								
									when x"B" => -- bitwise OR
											reg1:=reg1 OR reg2;								
									when x"C" => -- check reg1 == 0
											if(reg1=x"00000000") then
												z:='1';
											else
												z:='0';
											end if;
									when x"D"=>  -- switch memory
											if(instrx(3 downto 0)="0001") then
												s_ram_selector<="01";
													
											else
												s_ram_selector<="00";
													
											end if;
									when x"E"=> --shifts
										case instrx(3 downto 0) is
												when x"0" => -- left bit shift
														reg1:=reg1(30 downto 0) & "0";			
												when x"B" => -- left bit shift get LOW->HIGH
														reg1:=reg1(15 downto 0) & "0000000000000000";	
												when x"1" => -- right bit shift
														reg1:="0"&reg1(31 downto 1);	
												when x"A" => -- right 16 bit shift get HIGH->LOW
														reg1:="0000000000000000"&reg1(31 downto 16);			
												when x"2" => -- left cycle bit shift
														reg1:=reg1(30 downto 0) & reg1(31);			
												when x"3"=> -- right cycle bit shift
														reg1:=reg1(0)&reg1(31 downto 1) ;			
												when x"4"=> -- bitwise xor
														reg1:=reg1 xor reg2 ;	
												when others => null;
										end case;
									when others=> null;
								end case;
											
							
						when x"B0"|x"B1"|x"B2"|x"B3"|x"B4"|x"B5"|x"B6"|x"B7"|x"B8"|x"B9"|x"BA"|x"BB"|x"BC"|x"BD"|x"BE"|x"BF"=> -- jump operation
							if(instrx(11 downto 9)="000") then
								code_pointer:=conv_integer(instrx(8 downto 0));							
							elsif (instrx(11 downto 9)="001") then -- jump equal operation
								if(z='1') then
									code_pointer:=conv_integer(instrx(8 downto 0));							
								end if;
							elsif (instrx(11 downto 9)="010") then -- jump not equal operation
								if(z='0') then
									code_pointer:=conv_integer(instrx(8 downto 0));							
								end if;
							elsif (instrx(11 downto 9)="100") then -- jump to reg1								
								code_pointer:=conv_integer(reg1);															
							elsif (instrx(11 downto 9)="101") then -- jump to reg1 equal operation
								if(z='1') then
									code_pointer:=conv_integer(reg1);							
								end if;
							elsif (instrx(11 downto 9)="110") then -- jump to reg1 not equal operation  "BC"
								if(z='0') then
									code_pointer:=conv_integer(reg1);							
								end if;							
							end if;
								
						when x"AE" => --copy accumulator to reg1
								reg1:=accum;	
						
						-- Extended commnd
						when x"E0"=> --set vRegReadAddr =reg1
									--vRegReadAddr:=conv_integer(reg1(8 downto 0));														
									vRegReadAddr:=conv_integer(reg1);														
									
						when  x"EB"=> 								
						--add operations
														
										reg1:=reg1-1;									
						when  x"EA"=> 				
								
										reg1:=reg1+1;
								
						--when  x"E1"=> 				
						--shift operations
								
							--			reg1:=reg1(23 downto 0)&s_rom_q(7 downto 0);
							
							
								
						when x"90"=>  -- nop operations
						
						when  x"80"=>  -- output port operations
								case instrx(7 downto 0) is
									when x"0A" =>
										outpA<=reg1;
									when x"0B" =>
										outpB<=reg1;
									when x"0C" =>
										outpC<=reg1;
									when x"0D" =>
										outpD<=reg1;
									when x"0E" =>
										outpE<=reg1;
									when x"0F" =>
										outpF<=reg1;
									when others =>
								end case;
						when x"81"=>  -- input port operations
								case instrx(7 downto 0) is
									when x"0A"=>
										reg1:=inpA;
									when x"0B"=>
										reg1:=inpB;
									when x"0C"=>
										reg1:=inpC;																
									when others =>									
								end case;
								
						when x"82"=>  -- mov to z flag cmdready								
									z:=latch2_cmd_ready;
								
						when x"AF"=>
							
				  when others => 
				end case;
					
						
				return_state<=INST_FETCH;		
				
				
				case instrx(15 downto 8) is
					when x"D8"|x"A1"=> 
					when others=>						
						delay:=1;
						--state<=INST_FETCH;		
						state<=STATE4;
				end case;
				
					--elsif v3=3 then
					
					--elsif v3=2 then
					--	if(s_rom_q(15 downto 8)=x"AF") then--terminate execution
						--	return_state<=RST;	
						--else
						--	return_state<=INST_FETCH;	
						--end if;
								
						--delay:=3;
						--state<=STATE4;

					--end if;				
						
					--	v3:=v3+1;
					
							
	--------------------------------------------								
				when READ_REGISTER=>
				
				if v4=1 then
						s_ram_addr_2<=vRegReadAddr+vRegReadCount;
					elsif  v4=4 then
						reg1(7 downto 0):=s_ram_q;								
					elsif v4=5 then
						if(vRegReadCount=0) then									
								state<=return_state;
							end if;			
					elsif v4=6 then
						reg1(31 downto 8):=reg1(23 downto 0);											
					elsif v4=7 then
						vRegReadCount:=vRegReadCount-1;								
					elsif v4=8 then
						v4:=0;						
				end if;
				
					v4:=v4+1;
	--------------------------------------------								
				when READ_VARIABLE=>		
					--s_rom_addr<=code_pointer;				
					s_varram_addr<=vRegReadAddr;
					
					if v4=3 then	
							reg1:=s_varram_q;		
							state<=return_state;						
					end if;					
					v4:=v4+1;	
	--------------------------------------------								
				when WRITE_VARIABLE=>
					--s_rom_addr<=code_pointer;		
				if v4=1 then
						s_varram_addr<=vRegReadAddr;						
					elsif v4=3 then
						s_varram_we<='1';						
					elsif  v4=6 then
						s_varram_data<=reg1;
					elsif v4=8 then						
						s_varram_we<='0';
						state<=return_state;
				end if;				
					v4:=v4+1;					
				
--				if v4=0 then
--						s_varram_addr<=vRegReadAddr;											
--						s_varram_we<='1';						
--						s_varram_data<=reg1;
--				elsif v4=1 then
--						s_varram_we<='0';
--						state<=return_state;
--				end if;				
--					v4:=v4+1;	
	--------------------------------------------			
	when FIFO_READ_WAIT=>
		v4:=0;
		if(fifo_empty='0')then
			state<=FIFO_READ;
		end if;
	when FIFO_READ=>
				
				if v4=1 then
						fifo_rd<='1';						
					elsif v4=2 then
						fifo_rd<='0';
					elsif  v4=4 then
						reg1:=fifo_data;
					elsif v4=6 then						
						state<=return_state;
				end if;				
					v4:=v4+1;	
	
	when SFIFO_READ=>
				
				if v4=1 then
						s_sf_rd<='1';						
					elsif v4=2 then
						s_sf_rd<='0';
					elsif  v4=4 then
						reg1:=s_sf_q;
					elsif v4=6 then						
						state<=return_state;
				end if;				
					v4:=v4+1;	
	when SFIFO_WRITE=>
				
				if v4=1 then
						s_sf_data<=reg1;
						s_sf_wr<='1';						
					elsif v4=2 then
						s_sf_wr<='0';
					elsif  v4=3 then						
						state<=return_state;
				end if;				
					v4:=v4+1;	
--------------------------------------------								
				when READ_VARIABLE_SDRAM=>
				--sdram_s1_address<=std_logic_vector(to_unsigned(vRegReadAddr, sdram_s1_address'length));
				sdram_s1_address<=reg3(21 downto 0);
						sdram_s1_read_n<='0';
					sdram_s1_write_n<='1';
						state<=SDRAM_WAIT;
			
				when SDRAM_WAIT=>
				if(sdram_s1_waitrequest='0')then
							sdram_s1_read_n<='1';
							sdram_s1_write_n<='1';
							state<=SDRAM_WAIT2;
							
						end if;
				when SDRAM_WAIT2=>
				if(sdram_s1_readdatavalid='1')then
						
									state<=SDRAM_WAIT3;
									reg1:=x"0000"&sdram_s1_readdata;
							
								
						end if;
				when SDRAM_WAIT3=>
					sdram_s1_read_n<='1';
						sdram_s1_write_n<='1';
						state<=return_state;
	--------------------------------------------		
--------------------------------------------								
				when WRITE_VARIABLE_SDRAM=>
					--sdram_s1_address<=std_logic_vector(to_unsigned(vRegReadAddr, sdram_s1_address'length));
					sdram_s1_address<=reg3(21 downto 0);
					sdram_s1_writedata<=reg1(15 downto 0);
					
					
					sdram_s1_read_n<='1';
					sdram_s1_write_n<='0';				
						state<=SDRAM_WAIT4;
			
				when SDRAM_WAIT4=>
			if(sdram_s1_waitrequest='0')then						
						sdram_s1_read_n<='1';
						sdram_s1_write_n<='1';
						
						
						if(cntr3=1)then
								state<=return_state;							
								cntr3:=0;
							else
								state<=WRITE_VARIABLE_SDRAM;
								cntr3:=1;
							end if;
							
							
						
					end if;
			
	--------------------------------------------	
				when STATE4=>
					--s_rom_addr<=code_pointer;		
					delay:=delay-1;
					if(delay=0) then
						state<=return_state;
					end if;
					
	--------------------------------------------
--	when STATE1 =>
--		vOutp:="0001";
--		delay:=30e6;
--		return_state<=STATE2;
--		state<=STATE4;
--		
--		
--					
--	--------------------------------------------
--	when STATE2 =>
--	vOutp:="0010";
--	delay:=40e6;
--		return_state<=STATE3;
--		state<=STATE4;
--		
--					
--	--------------------------------------------
--	when STATE3 =>
--	vOutp:="0011";
--	delay:=50e6;
--		return_state<=RST;
--		state<=STATE4;
--	
	
			when		FPU_INT2FLOAT=>
							s_i2f_input_a<=reg1;
							s_i2f_input_a_stb<='1';
							state<=FPU_INT2FLOAT_WAIT;
							s_i2f_output_z_ack<='0';
			when		FPU_INT2FLOAT_WAIT=>
						s_i2f_input_a_stb<='0';
						if(s_i2f_output_z_stb='1')then
							s_i2f_output_z_ack<='1';
							reg1:=s_i2f_output_z;
							state<=return_state;
							end if;
	when FPU_FLOAT2INT=>
							s_f2i_input_a<=reg1;
							s_f2i_input_a_stb<='1';
							state<=FPU_FLOAT2INT_WAIT;
							s_f2i_output_z_ack<='0';
							
		
	when FPU_FLOAT2INT_WAIT=>
						s_f2i_input_a_stb<='0';
						if(s_f2i_output_z_stb='1')then
							s_f2i_output_z_ack<='1';
							reg1:=s_f2i_output_z;
							state<=return_state;
							end if;
when	FPU_MUL=>
		s_mul_output_z_ack<='0';
				s_mul_input_a<=reg1;
				s_mul_input_b<=reg2;
				s_mul_input_a_stb<='1';
				s_mul_input_b_stb<='1';
				state<=FPU_MUL_B;
	when FPU_MUL_B=>
						if(s_mul_input_b_ack='1')then
								state<=FPU_MUL_WAIT;
						end if;
when	FPU_MUL_WAIT=>
				s_mul_input_a_stb<='0';
				s_mul_input_b_stb<='0';
				if(s_mul_output_z_stb='1')then
						s_mul_output_z_ack<='1';
						reg1:=s_mul_output_z;
						state<=return_state;
				end if;
	when FPU_DIV=>
				state<=FPU_DIV_WAIT;
	when FPU_DIV_WAIT=>
				state<=return_state;
				
				
	when FPU_ADD=>
				s_adder_output_z_ack<='0';
				s_adder_input_a<=reg1;
				s_adder_input_b<=reg2;
					s_adder_input_a_stb<='1';
					s_adder_input_b_stb<='1';
					state<=FPU_ADD_B;
	when FPU_ADD_B=>
						if(s_adder_input_b_ack='1')then
							state<=FPU_ADD_WAIT;
						end if;
						
	when FPU_ADD_WAIT=>
			s_adder_input_a_stb<='0';
			s_adder_input_b_stb<='0';
			if(s_adder_output_z_stb='1')then
				s_adder_output_z_ack<='1';
				reg1:=s_adder_output_z;
				state<=return_state;
		end if;
	--------------------------------------------
	when READ_SECTOR=>
				aCounter:=0;
				s_address<=vAddress;
					s_rd<='1';	
			if(s_read_flag='1')		 then					
					state<=READ_SECTOR_WAIT;
			end if;
---------------------------------------------------
				when READ_SECTOR_WAIT=>		
			s_rd<='0';
				state<=FINISH_READ_SECTOR;
				

---------------------------------------------------
				when FINISH_READ_SECTOR=>
				aCounter:=aCounter+1;
				if(s_read_flag='0' ) then
					
					counter<=aCounter;
					state<=return_state;										
				end if;
---------------------------------------------------
	when WRITE_SECTOR=>
				aCounter:=0;
				s_address<=vAddress;
				s_wr<='1';				
				if(s_write_flag='1')		 then					
					state<=WRITE_SECTOR_WAIT;
			end if;	
---------------------------------------------------
		when WRITE_SECTOR_WAIT=>		
				s_wr<='0';
				state<=FINISH_WRITE_SECTOR;			

---------------------------------------------------
		when FINISH_WRITE_SECTOR=>
				aCounter:=aCounter+1;
					if(s_write_flag='0' ) then
					
					counter<=aCounter;
					state<=return_state;										
				end if;										
				
---------------------------------------------------
		when RELAX=>		
				vAddress:=vAddress+512;		
				state<=STATE4;
				return_state<=RST;	
end case;


end if;
outp<=vOutp;
regs1<=reg1;
regs2<=reg2;
regs3<=reg3;



end process;	
			

ram1:varram
	port  map	(
	clock	=>clk,
		
		address	=>conv_Std_logic_vector(s_varram_addr_1,10),
		data	=>s_varram_data_1,
		wren		=>s_varram_we_1,
		q		=>s_varram_q_1		
		);
		

ram0:ram1p
	port  map	(
	clock	=>clk,
		
		address	=>conv_Std_logic_vector(s_ram_addr,9),
		data	=>s_ram_data,
		wren		=>s_ram_we,
		q		=>s_ram_q		
		);
		
--s_din<=s_ram_q;

s_ram_addr<=s_ram_addr_1 when (s_read_flag='1' or s_write_flag='1') else
				s_ram_addr_2;
	
s_ram_we<=s_ram_we_1 when s_read_flag='1' else
				s_ram_we_2;
	
	s_ram_we_2<='0';
	
rom0:progrom
port  map	(
	clock	=>clk,		
		address	=>conv_Std_logic_vector(s_rom_addr,12),
		--rden	=>s_progrom_rden,
		q		=>s_rom_q		
);
		
		
outflag<='1' when (counter>0 or regs1>0 
or regs2>0 or regs3>0
  or s_ram_q>0 ) else '0';




end rtl;
