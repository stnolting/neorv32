library vunit_lib;
context vunit_lib.com_context;

package uart_rx_pkg is
  constant check_uart_msg : msg_type_t := new_msg_type("check_uart");

  procedure check_uart(
    signal net : inout network_t;
    constant actor : in actor_t;
    constant data : in string);
end package uart_rx_pkg;

package body uart_rx_pkg is
  procedure check_uart(
    signal net : inout network_t;
    constant actor : in actor_t;
    constant data : in string) is
    variable msg : msg_t;
  begin
    msg := new_msg(check_uart_msg);
    push(msg, data);
    send(net, actor, msg);
  end;

end package body uart_rx_pkg;
