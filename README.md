# firmware mirror

OpenPuck release `.uf2` files, mirrored from the GitHub release assets because
raw.githubusercontent.com serves files WITH CORS headers while the release-asset CDN does
not — this is what lets the WebUSB config panel download and flash releases in-browser.
Maintained automatically by `.github/workflows/release.yml`; do not edit by hand.
