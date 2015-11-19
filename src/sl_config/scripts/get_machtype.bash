#!/bin/bash
# Remove everything following the "-" in the MACHTYPE 
export MACHTYPE=$(echo $MACHTYPE | sed 's/-.*//') 

# Append mac to the MACHTYPE if needed 
if uname -s | grep Darwin > /dev/null 2>&1 && [ "$MACHTYPE" = "i386" -o "$MACHTYPE" = "x86_64" ] 
then 
    export MACHTYPE="$MACHTYPE"mac 
fi 

# Append xeno to the MACHTYPE if needed (and prevent duplicates) 
if uname -r | grep ipipe > /dev/null 2>&1 && echo $MACHTYPE | grep -v xeno >/dev/null 2>&1
then 
    export MACHTYPE="$MACHTYPE"xeno 
fi 

echo $MACHTYPE
