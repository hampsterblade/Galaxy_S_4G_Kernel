#!/sbin/sh
if [ -f /system/media/bootanimation.zip -o -f /data/media/bootanimation.zip ]; then
  /sbin/bootanimation
else
  /system/bin/samsungani
fi;
