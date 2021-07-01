library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;
use vunit_lib.sync_pkg.all;

package uart_rx_pkg is
  constant check_uart_msg : msg_type_t := new_msg_type("check_uart");

  type uart_rx_t is record
    p_baud_val : real;
    p_logger : logger_t;
    p_actor : actor_t;
  end record;

  impure function new_uart_rx(
    baud_val : real;
    logger : logger_t := null_logger;
    actor : actor_t := null_actor) return uart_rx_t;

  function as_sync(handle : uart_rx_t) return sync_handle_t;

  procedure check_uart(
    signal net : inout network_t;
    constant handle : in uart_rx_t;
    constant data : in string);
end package uart_rx_pkg;

package body uart_rx_pkg is
  constant uart_rx_logger  : logger_t  := get_logger("neorv32_lib:uart_rx_pkg");

  impure function new_uart_rx(
    baud_val : real;
    logger : logger_t := null_logger;
    actor : actor_t := null_actor) return uart_rx_t is
    variable result : uart_rx_t;
  begin
    result.p_baud_val := baud_val;
    result.p_logger := logger when logger /= null_logger else uart_rx_logger;
    result.p_actor := actor when actor /= null_actor else new_actor;

    return result;
  end;

  function as_sync(handle : uart_rx_t) return sync_handle_t is
  begin
    return handle.p_actor;
  end;

  procedure check_uart(
    signal net : inout network_t;
    constant handle : in uart_rx_t;
    constant data : in string) is
    variable msg : msg_t;
  begin
    msg := new_msg(check_uart_msg);
    push(msg, data);
    send(net, handle.p_actor, msg);
  end;

end package body uart_rx_pkg;
