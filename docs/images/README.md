# Images

Photos for the README.

Resize before committing. A phone photo is 3-5 MB and a README never displays it
above about 1500 px wide, so it is several megabytes of repository for no visible
difference. This shrinks one in place:

```bash
sips -Z 1500 photo.jpg
```

Git keeps every version of a binary forever, so a re-cropped 4 MB photo costs
another 4 MB in the history rather than replacing the first.
