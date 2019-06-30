library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

entity rt_reconf is
	port (
		-- OSIF FIFO ports
		OSIF_Sw2Hw_Data    : in  std_logic_vector(31 downto 0);
		OSIF_Sw2Hw_Empty   : in  std_logic;
		OSIF_Sw2Hw_RE      : out std_logic;

		OSIF_Hw2Sw_Data    : out std_logic_vector(31 downto 0);
		OSIF_Hw2Sw_Full    : in  std_logic;
		OSIF_Hw2Sw_WE      : out std_logic;

		-- MEMIF FIFO ports
		MEMIF_Hwt2Mem_Data    : out std_logic_vector(31 downto 0);
		MEMIF_Hwt2Mem_Full    : in  std_logic;
		MEMIF_Hwt2Mem_WE      : out std_logic;

		MEMIF_Mem2Hwt_Data    : in  std_logic_vector(31 downto 0);
		MEMIF_Mem2Hwt_Empty   : in  std_logic;
		MEMIF_Mem2Hwt_RE      : out std_logic;

		HWT_Clk    : in  std_logic;
		HWT_Rst    : in  std_logic;
		HWT_Signal : in  std_logic;
		
		-- General purpose output ports offered by each HWT slot. Not used in this module.
		GP_OUT : out std_logic_vector(7 downto 0)
	);
end entity rt_reconf;

architecture module_1 of rt_reconf is
	type STATE_TYPE is (
					STATE_INIT,
					STATE_GET_ADDR,STATE_READ,
					STATE_PROCESSING,
					STATE_WRITE,STATE_ACK,STATE_THREAD_EXIT);

	component process_module is
		port (
			ap_clk : IN STD_LOGIC;
			ap_rst : IN STD_LOGIC;
			ap_start : IN STD_LOGIC;
			ap_done : OUT STD_LOGIC;
			ap_idle : OUT STD_LOGIC;
			ap_ready : OUT STD_LOGIC;
			ram_Addr_A : OUT STD_LOGIC_VECTOR (31 downto 0);
			ram_EN_A : OUT STD_LOGIC;
			ram_WEN_A : OUT STD_LOGIC_VECTOR (3 downto 0);
			ram_Din_A : OUT STD_LOGIC_VECTOR (31 downto 0);
			ram_Dout_A : IN STD_LOGIC_VECTOR (31 downto 0);
			ram_Clk_A : OUT STD_LOGIC;
			ram_Rst_A : OUT STD_LOGIC 
		);
	end component;

	-- IMPORTANT: define size of local RAM here!!!! 
	constant C_LOCAL_RAM_SIZE          : integer := 64;
	constant C_LOCAL_RAM_ADDRESS_WIDTH : integer := integer(ceil(log2(real(C_LOCAL_RAM_SIZE))));
	constant C_LOCAL_RAM_SIZE_IN_BYTES : integer := 4*C_LOCAL_RAM_SIZE;

	type LOCAL_MEMORY_T is array (0 to C_LOCAL_RAM_SIZE-1) of std_logic_vector(31 downto 0);

	-- Definition of MBOX resource IDs according to build.cfg
	constant MBOX_RECV  : std_logic_vector(31 downto 0) := x"00000000";
	constant MBOX_SEND  : std_logic_vector(31 downto 0) := x"00000001";

	-- Signals and interfaces for ReconOS
	signal addr     : std_logic_vector(31 downto 0);
	signal len      : std_logic_vector(31 downto 0);
	signal state    : STATE_TYPE;
	signal i_osif   : i_osif_t;
	signal o_osif   : o_osif_t;
	signal i_memif  : i_memif_t;
	signal o_memif  : o_memif_t;
	signal i_ram    : i_ram_t;
	signal o_ram    : o_ram_t;

	-- BRAM interface for HLS processing module
	signal o_RAMAddr_process_module : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMData_process_module : std_logic_vector(0 to 31);
	signal o_RAMWE_process_module   : std_logic;
	signal i_RAMData_process_module : std_logic_vector(0 to 31);
	signal ram_Addr_A_intermediate : std_logic_vector(31 downto 0);
	signal ram_WEN_A_intermediate  : std_logic_vector(3 downto 0);

	-- BRAM interface for ReconOS
	signal o_RAMAddr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMAddr_reconos_2 : std_logic_vector(0 to 31);
	signal o_RAMData_reconos   : std_logic_vector(0 to 31);
	signal o_RAMWE_reconos     : std_logic;
	signal i_RAMData_reconos   : std_logic_vector(0 to 31);

	constant o_RAMAddr_max : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) := (others=>'1');

	shared variable local_ram : LOCAL_MEMORY_T;

	signal ignore   : std_logic_vector(31 downto 0);
	signal status : std_logic_vector(31 downto 0);

	signal proc_start   : std_logic;
	signal proc_done	: std_logic;

begin
	-- local dual-port RAM
	local_ram_ctrl_1 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then
			if (o_RAMWE_reconos = '1') then
				local_ram(to_integer(unsigned(o_RAMAddr_reconos))) := o_RAMData_reconos;
			else
				i_RAMData_reconos <= local_ram(to_integer(unsigned(o_RAMAddr_reconos)));
			end if;
		end if;
	end process;

	local_ram_ctrl_2 : process (HWT_Clk) is
	begin
		if (rising_edge(HWT_Clk)) then		
			if (o_RAMWE_process_module = '1') then
				local_ram(to_integer(unsigned(o_RAMAddr_process_module))) := o_RAMData_process_module;
			else
				i_RAMData_process_module <= local_ram(to_integer(unsigned(o_RAMAddr_process_module)));
			end if;
		end if;
	end process;

	-- Instantiate HLS processing module and connect BRAM and control interfaces
	process_module_i : process_module
		port map (		
			ap_clk       => HWT_Clk,
			ap_rst       => HWT_Rst,
			ap_start     => proc_start,
			ap_done      => proc_done,
			--ap_idle
			--ap_ready
			ram_Addr_A   => ram_Addr_A_intermediate, 
			--ram_EN_A
			ram_WEN_A    => ram_WEN_A_intermediate, 
			ram_Din_A    => o_RAMData_process_module,
			ram_Dout_A   => i_RAMData_process_module
			--ram_Clk_A
			--ram_Rst_A
	);

	o_RAMAddr_process_module <= ram_Addr_A_intermediate(C_LOCAL_RAM_ADDRESS_WIDTH-1+2 downto 0+2); --only use required addr width
	o_RAMWE_process_module <= ram_WEN_A_intermediate(0) or ram_WEN_A_intermediate(1) or ram_WEN_A_intermediate(2) or ram_WEN_A_intermediate(3); --combine 4 byte wise WE signals

	-- ReconOS initilization
	osif_setup (
		i_osif,
		o_osif,
		OSIF_Sw2Hw_Data,
		OSIF_Sw2Hw_Empty,
		OSIF_Sw2Hw_RE,
		OSIF_Hw2Sw_Data,
		OSIF_Hw2Sw_Full,
		OSIF_Hw2Sw_WE
	);

	memif_setup (
		i_memif,
		o_memif,
		MEMIF_Mem2Hwt_Data,
		MEMIF_Mem2Hwt_Empty,
		MEMIF_Mem2Hwt_RE,
		MEMIF_Hwt2Mem_Data,
		MEMIF_Hwt2Mem_Full,
		MEMIF_Hwt2Mem_WE
	);
	
	ram_setup (
		i_ram,
		o_ram,
		o_RAMAddr_reconos_2,
		o_RAMData_reconos,
		i_RAMData_reconos,
		o_RAMWE_reconos
	);
	
	o_RAMAddr_reconos(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) <= o_RAMAddr_reconos_2((32-C_LOCAL_RAM_ADDRESS_WIDTH) to 31);
	
	-- os and memory synchronisation state machine
	reconos_fsm: process (HWT_Clk,HWT_Rst,o_osif,o_memif,o_ram) is
		variable done : boolean;
	begin
		if HWT_Rst = '1' then
			osif_reset(o_osif);
			memif_reset(o_memif);
			ram_reset(o_ram);
			state <= STATE_INIT;
			done := False;
			addr <= (others => '0');
			len <= std_logic_vector(TO_UNSIGNED(1*4, 32)); -- Define how many bytes (or words) to transfer b/w RAM and BRAM per Operation
			proc_start <= '0';
	  
		elsif rising_edge(HWT_Clk) then
			case state is
				when STATE_INIT =>
					if HWT_SIGNAL = '1' then -- Suspend thread if SIGNAL is set, this is checked in every state
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_read(i_osif, o_osif, ignore, done); -- Read a word from the OSIF, this must be done at init. of every HWT
						if done then
							state <= STATE_GET_ADDR;
						end if;
					end if;

				-- get address via mbox: the data will be copied from this address to the local ram in the next states
				when STATE_GET_ADDR =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_mbox_tryget(i_osif, o_osif, MBOX_RECV, addr, status, done);
						if done then
							if status = x"00000000" then
								state <= STATE_GET_ADDR;
							else
								state <= STATE_READ;
							end if;
						end if;
					end if;
				
				-- copy data from main memory to local memory
				when STATE_READ =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_read(i_ram,o_ram,i_memif,o_memif,addr(31 downto 2) & "00",X"00000000",len,done);
						if done then
							proc_start <= '1';
							state <= STATE_PROCESSING;
						end if;
					end if;
				
				-- wait until HLS processing module has finished the operation
				when STATE_PROCESSING =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						proc_start <= '0';
		
						if proc_done = '1' then
							state  <= STATE_WRITE;
						end if;
					end if;
			
				-- copy data from local memory to main memory
				when STATE_WRITE =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						memif_write(i_ram,o_ram,i_memif,o_memif,X"00000000",addr,len,done);
						if done then
							state <= STATE_ACK;
						end if;
					end if;
				
				-- send mbox that signals that the processing is finished -> then wait for next input data
				when STATE_ACK =>
					if HWT_SIGNAL = '1' then
						osif_reset(o_osif);
						memif_reset(o_memif);
						state <= STATE_THREAD_EXIT;					
					else
						osif_mbox_put(i_osif, o_osif, MBOX_SEND, addr, ignore, done);
						if done then state <= STATE_GET_ADDR; end if;
					end if;

				-- thread exit
				when STATE_THREAD_EXIT =>
					osif_thread_exit(i_osif,o_osif);
			
			end case;
		end if;
	end process;

end architecture module_1;
