#!/bin/bash

echo -n checking for \"UseDNS no\" in /etc/ssh/sshd_config...
if grep -Fxq "UseDNS no" /etc/ssh/sshd_config
then
    echo found.
else
    echo not found.
    echo -n Adding...
    if echo UseDNS no >> /etc/ssh/sshd_config
    then
	echo done.
    else
	echo error!
    fi
fi

echo -n Setting Timezone to USA/Detroit...
sudo ln -sf /usr/share/zoneinfo/America/Detroit /etc/localtime
echo done.
date