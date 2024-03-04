#!/bin/bash
ls bayer*.jpg | sed -n 's/^bayer\([0-9]\+\).*$/..\/Bayer2RGB\/Bayer2RGB bayer\1.jpg rgb\1.png/gp' >go
