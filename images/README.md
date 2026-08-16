# Images

Photos of the build. Referenced from the root `README.md`.

Filenames are the camera's originals. Two were full resolution (4032 px) and
have been resized to 1500 — GitHub never renders wider than about that in a
README, so anything larger is repository weight for no visible difference.

Checked before publishing: none carry GPS EXIF.

Adding more: resize first, because git keeps every version of a binary forever.
A re-cropped 4 MB photo costs another 4 MB in the history rather than replacing
the original.

```bash
sips -Z 1500 images/*.jpg
```

Avoid spaces in filenames — every markdown link to one needs `%20` escaping.
