#!/usr/bin/env bash
/usr/local/bin/docker buildx build . -t tetsuzawa/ahrs-pi:0.0.2 --platform linux/arm64/v8
