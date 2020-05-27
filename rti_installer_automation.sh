#!/usr/bin/expect
set timeout 360
spawn ./rti_connext_dds-6.0.0-eval-x64Linux4gcc7.3.0.run --prefix /opt/rti_connext_dds-6.0.0
send "\r"
send "\r"
send "\r"
send "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\ry"
send "\r"
send "\r"
send "\r"
expect "Press \[Enter\] to continue"
send "\r"
send "\r"
