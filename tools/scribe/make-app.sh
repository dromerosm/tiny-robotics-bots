#!/bin/sh
# Build Scribe.app (release) into .work/, signed ad hoc for this Mac.
#   ./make-app.sh && open .work/Scribe.app
set -eu
cd "$(dirname "$0")"
swift build -c release --product Scribe
APP=.work/Scribe.app
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp .build/release/Scribe "$APP/Contents/MacOS/Scribe"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>Scribe</string>
  <key>CFBundleDisplayName</key><string>Scribe</string>
  <key>CFBundleIdentifier</key><string>es.diegoromero.scribe</string>
  <key>CFBundleExecutable</key><string>Scribe</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>0.1</string>
  <key>CFBundleVersion</key><string>1</string>
  <key>LSMinimumSystemVersion</key><string>15.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>LSApplicationCategoryType</key><string>public.app-category.productivity</string>
</dict></plist>
PLIST
codesign --force --sign - "$APP"
echo "built $APP"
