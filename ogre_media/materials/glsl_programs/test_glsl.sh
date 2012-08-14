if [[ "$1" == *.vert ]]; then
  cgc -oglsl -profile arbvp1 -strict -nocode $1
elif [[ "$1" == *.frag ]]; then
  cgc -oglsl -profile arbfp1 -strict -nocode $1
elif [[ "$1" == *.geom ]]; then
  cgc -oglsl -profile gs_5_0 -strict -nocode $1
else
  echo "Unknown shader type!"
  exit 1
fi

