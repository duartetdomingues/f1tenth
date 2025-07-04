#!/usr/bin/expect -f

set timeout 20
set MAC "40:8E:2C:A0:F8:0D"

spawn bluetoothctl
expect "#"

send "remove $MAC\r"
expect "#"

sleep 2

send "scan on\r"
set found 0
for {set i 0} {$i < 10} {incr i} {
    expect {
        -re "$MAC" {
            puts "✅ Dispositivo encontrado: $MAC"
            set found 1
            break
        }
        timeout {
            puts "⏳ À espera do dispositivo... ($i)"
        }
    }
}
if {$found == 0} {
    puts "❌ Dispositivo não apareceu. A sair."
    send "exit\r"
    exit 1
}

sleep 1
send "scan off\r"
expect "#"
sleep 1

send "connect $MAC\r"
expect {
    "Connection successful" { puts "✅ Ligado com sucesso" }
    timeout { puts "❌ Timeout ao ligar"; exit 1 }
}

send "exit\r"
expect eof
