import { initializeApp } from 'https://www.gstatic.com/firebasejs/9.18.0/firebase-app.js';
import {
  getAuth,
  signInWithPopup,
  GoogleAuthProvider,
  onAuthStateChanged,
  signOut
} from 'https://www.gstatic.com/firebasejs/9.18.0/firebase-auth.js';
import { getFirestore } from 'https://www.gstatic.com/firebasejs/9.18.0/firebase-firestore.js';

// Firebase configuration
const firebaseConfig = {
  apiKey: "",
  authDomain: "",
  projectId: "",
  storageBucket: "",
  messagingSenderId: "",
  appId: "",
  measurementId: ""
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const auth = getAuth(app);
const db = getFirestore(app);

// Make Firebase services globally available (optional)
window.auth = auth;
window.db = db;

// Google Auth provider
const provider = new GoogleAuthProvider();

// On DOMContentLoaded, attach event listeners
document.addEventListener("DOMContentLoaded", () => {
  // 1. GOOGLE SIGN-IN BUTTON (on index.html)
  const googleSignInButton = document.getElementById("google-signin");
  if (googleSignInButton) {
    googleSignInButton.addEventListener("click", async () => {
      try {
        const result = await signInWithPopup(auth, provider);
        const user = result.user;
        console.log("User signed in with Google:", user);
        
        document.getElementById("login-message").innerText = "Google login successful!";
        // Redirect to dashboard after a short delay
        setTimeout(() => {
          window.location.href = "dashboard.html";
        }, 1000);
      } catch (error) {
        console.error("Google login failed:", error.message);
        alert("Google login failed: " + error.message);
      }
    });
  } else {
    // Not on the login page (index.html), so no sign-in button
    // console.log("No Google Sign-In button found; this is likely not the login page.");
  }

  // 2. LOGOUT BUTTON (on dashboard.html, test.html, contact.html, etc.)
  const logoutButton = document.getElementById("logout-button");
  if (logoutButton) {
    logoutButton.addEventListener("click", async () => {
      try {
        await signOut(auth);
        alert("You have been logged out.");
        window.location.href = "index.html";
      } catch (error) {
        console.error("Logout error:", error.message);
        alert("Failed to log out: " + error.message);
      }
    });
  } else {
    // console.log("No logout button on this page.");
  }
});

// Monitor auth state changes (optional for debugging)
onAuthStateChanged(auth, (user) => {
  if (user) {
    console.log("Authenticated user:", user.email);
  } else {
    console.warn("No user is authenticated.");
  }
});
