rm tmp/sc/*
poetry run python3 vis_het.py
ffmpeg -y -framerate 30 -start_number 20 -i tmp/sc/screenshot_%06d.png -vf "palettegen" /tmp/palette.png 
ffmpeg -y -framerate 30 -start_number 20 -i tmp/sc/screenshot_%06d.png -i /tmp/palette.png -filter_complex "paletteuse" output.gif