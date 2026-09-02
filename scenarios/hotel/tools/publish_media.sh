#!/usr/bin/env bash
# Copies the film and the diagrams into the site, and cuts the link-preview
# image out of the film itself.
set -euo pipefail
REPO="${HOME}/Projects/Epistemic-Robotics"
SITE="${HOME}/Projects/er-pages"
LEAK="${1:-l3_suite}"

VIDEO="${REPO}/scenarios/hotel/out/hotel-${LEAK}.mp4"
[ -f "${VIDEO}" ] || { echo "no film at ${VIDEO}"; exit 1; }

mkdir -p "${SITE}/media"
cp "${VIDEO}" "${SITE}/media/hotel_run_l3.mp4"
for d in hotel-seam hotel-policy hotel-model; do
  cp "${REPO}/docs/img/${d}.svg" "${SITE}/media/${d}.svg"
done

# A frame from a third of the way in, where both robots are moving. The strip
# is far wider than the 1200x630 the link scrapers want, so it is scaled to
# width and padded rather than cropped: cropping it would cut one of the two
# panes out, and the pair is the picture.
ffmpeg -hide_banner -loglevel error -y \
  -ss "$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${VIDEO}" \
         | awk '{printf "%.0f", $1/3}')" \
  -i "${VIDEO}" -frames:v 1 \
  -vf "scale=1200:-2,pad=1200:630:0:(630-ih)/2:color=0x111111" -q:v 3 \
  "${SITE}/media/og-hotel-run.jpg"

ls -l "${SITE}/media/hotel_run_l3.mp4" "${SITE}/media/og-hotel-run.jpg"
