#! /usr/bin/env bash
pipe=./bt_restart
[ -p "$pipe" ] || mkfifo -m 0600 "$pipe" || exit 1
while :; do
    while read -r cmd; do
        if [ "$cmd" ]; then
            printf 'Restart Bluetooth \n'
            /etc/init.d/bluetooth restart
        fi
    done <"$pipe"
done