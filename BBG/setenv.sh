#!/usr/bin/env bash

VARS=( VX_SHADER_PATH= VX_FONT_PATH= CLASSPATH= )
PATHS=( $PWD/botgui/vx/shaders $PWD/botgui/vx/fonts \
        $CLASSPATH:$PWD/java/rob550.jar )

if [ ! -f ~/.bashrc ]; then
   cp /etc/skel/.bashrc ~/. 
fi

# just check if one of the commands isn't in there exactly
if [ ! -z "${VARS[1]}" ]
then
    export VX_SHADER_PATH=$PWD/botgui/vx/shaders
    export VX_FONT_PATH=$PWD/botgui/vx/fonts
    export CLASSPATH=$CLASSPATH:$PWD/java/rob550.jar
fi

for i in ${!VARS[@]};
do
    if ! grep -Fq "export ${VARS[$i]}" ~/.bashrc
    then
        echo "export ${VARS[$i]}${PATHS[$i]}" >> ~/.bashrc
    else
        sed -i '/${VARS[$i]}/c\${VARS[$i]}${PATHS[$i]}' ~/.bashrc
    fi
done
