# Todoes Web

A static web version of the sibling `../Todoes` LVGL todo UI. It keeps the same active todo screen shape: a dark header, item count, nested rows, add/edit/delete actions, expanded sub-todoes, and the same `Todoes` wording.

The app saves to Cloud Firestore when it is running on Firebase Hosting. Plain local previews fall back to `localStorage`.

## Run locally

Open `public/index.html` directly in a browser, or serve the folder with any static file server.

## Firebase Hosting

This project is ready for Firebase Hosting through `firebase.json`.

```powershell
firebase login
firebase use --add
firebase deploy --only hosting,firestore:rules
```

When prompted by `firebase use --add`, select or create the Firebase project you want to host this site under.

The current Firestore rules intentionally allow public read/write access to only `todoLists/main`. Do not use this for private data yet. The next auth step should replace that public rule with Google sign-in and a two-account allowlist.
