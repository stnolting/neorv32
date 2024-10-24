library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_sigcount is
    generic (
        DEBOUNCE_LIMIT : natural := 500000  -- debounce limit (default value)
    );
    port (
        clk_i        : in  std_ulogic;                        -- clock input
        rstn_i       : in  std_ulogic;                        -- reset input (active low)
        button_i     : in  std_ulogic;                        -- button input
        bus_req_i    : in  bus_req_t;                         -- bus request
        bus_rsp_o    : out bus_rsp_t                         -- bus response
        -- counter_o    : out std_ulogic_vector(31 downto 0)    -- counter output FOR DEBUG
    );
end entity neorv32_sigcount;

architecture rtl of neorv32_sigcount is

    signal debounced_button : std_ulogic := '0';              -- debounced button signal
    signal counter          : unsigned(31 downto 0) := (others => '0'); -- counter
    signal button_prev      : std_ulogic := '0';              -- previous state of the button
    signal debounce_counter  : natural := 0;                  -- debounce counter

begin

    -- Debounce process
    debounce: process(clk_i, rstn_i) 
    begin
        if (rstn_i = '0') then
            debounced_button <= '0';
            debounce_counter <= 0;
            button_prev <= '0';
        elsif rising_edge(clk_i) then
            if button_i /= button_prev then
                debounce_counter <= 0;  -- Reset counter if state changes
            else
                if debounce_counter < DEBOUNCE_LIMIT then
                    debounce_counter <= debounce_counter + 1;  -- Increment counter
                else
                    debounced_button <= button_i;  -- Update debounced state
                end if;
            end if;
            button_prev <= button_i;  -- Store the previous state
        end if;
    end process debounce;

    -- Edge detection and counter
    edge_detect: process(clk_i, rstn_i) 
    begin
        if (rstn_i = '0') then
            counter <= (others => '0');  -- Reset counter
        elsif rising_edge(clk_i) then
            if (debounced_button = '1' and button_prev = '0') then
                counter <= counter + 1;  -- Increment on rising edge
            end if;
        end if;
    end process edge_detect;

    -- Bus access process
    bus_access: process(clk_i, rstn_i) 
    begin
        if (rstn_i = '0') then
            bus_rsp_o <= (others => '0');  -- Reset bus response
        elsif rising_edge(clk_i) then
            -- Bus handshake
            bus_rsp_o.ack <= bus_req_i.stb;
            bus_rsp_o.err <= '0';
            if (bus_req_i.stb = '1') then
                if (bus_req_i.rw = '0') then  -- Read access only
                    bus_rsp_o.data <= std_logic_vector(counter);  -- Read counter value
                end if;  -- No write access, so do nothing for rw = '1'
            end if;
        end if;
    end process bus_access;

    -- Output counter FOR DEBUG
    -- counter_o <= std_logic_vector(counter);

end architecture rtl;
