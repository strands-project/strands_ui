#!/bin/bash
##########################################################################
# MARY TTS server
##########################################################################

# Set the Mary base installation directory in an environment variable:
#BINDIR="`dirname "$0"`"

BINDIR=`rospack find mary_tts`/marytts-5.0/bin

export MARY_BASE="`(cd "$BINDIR"/.. ; pwd)`"

java -showversion -ea -Xms40m -Xmx1g -cp "$MARY_BASE/lib/*" -Dmary.base="$MARY_BASE" -Dlog4j.logger.marytts=WARN,stderr marytts.server.Mary
