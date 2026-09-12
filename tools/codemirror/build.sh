#!/bin/sh
# Builds codemirror.js into the coding server's resources. Needs node and npm; the output is committed,
# so the Gradle build and CI never run this.
set -eu
cd "$(dirname "$0")"
if [ -f package-lock.json ]; then npm ci --no-audit --no-fund; else npm install --no-audit --no-fund; fi
OUT=../../TeamCode/src/test/resources/org/firstinspires/ftc/teamcode/sim/codemirror.js
VERSIONS=$(node -e 'const p=require("./package.json").devDependencies; console.log(Object.keys(p).filter(k=>k!=="esbuild").map(k=>k+"@"+p[k]).join(", "))')
npx esbuild entry.js --bundle --minify --format=iife --target=es2018 --legal-comments=none \
  --banner:js="/* CodeMirror for the coding server: $VERSIONS. Built by tools/codemirror/build.sh; edit entry.js there, not this file. */" \
  --outfile="$OUT"
ls -l "$OUT"
