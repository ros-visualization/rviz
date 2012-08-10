if [[ "$1" == *_vp* ]]; then
  cgc -oglsl -profile arbvp1 -strict -nocode $1
elif [[ "$1" == *_fp* ]]; then
  cgc -oglsl -profile arbfp1 -strict -nocode $1
else
  echo "Unknown shader type!"
  exit 1
fi

