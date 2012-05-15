#!/sbin/sh

if test -f /data/local/bootanimation.zip || test -f /system/media/bootanimation.zip; then
	/system/bin/bootanimation
else
	  /system/bin/samsungani
fi

