# Generate data sheet NEORV32.pdf from adoc sources using asciidoctor-pdf

#!/bin/bash

# Abort if any command returns != 0
set -e

thisdir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
asciidoctor-pdf -a pdf-theme=$thisdir/src_adoc/neorv32-theme.yml $thisdir/src_adoc/neorv32.adoc --out-file $thisdir/NEORV32.pdf
