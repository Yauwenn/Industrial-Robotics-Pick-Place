# Industrial-Robotics-Pick-Place

🚀 MATLAB + GitHub Quick Start Guide for Our Team
Hey team! To make sure we don't accidentally delete each other’s code, we are using GitHub to manage our Pick and Place Robot simulation. You don't need to install anything on your computer; we will do everything directly inside the browser using MATLAB Online.

Here is your step-by-step guide to getting set up and submitting your work.

🛠️ PHASE 1: First-Time Setup (Do this once)
Step 1: Create your Accounts

Make a free account on GitHub.com.

Log into MATLAB Online using your university email.


Step 2: Generate your GitHub "Password" (Token)
GitHub doesn't let you use your normal password in MATLAB anymore. You need a "Token".

Go to GitHub -> Click your profile picture (top right) -> Settings.

Scroll down the left menu to Developer settings -> Personal access tokens -> Tokens (classic).

Click Generate new token (classic).

Name it "MATLAB", set expiration to 90 days, and CHECK THE BOX next to repo.

Click Generate at the bottom.

COPY the long ghp_... password it gives you and save it somewhere safe! You will need this every time you upload your code.


Step 3: Download (Clone) Our Project to your MATLAB

In MATLAB Online, make sure you are in your root folder (MATLAB Drive).

Look at the Command Window at the bottom of the screen. Type this exactly and hit Enter:
!git clone [INSERT_YOUR_GITHUB_REPO_URL_HERE]

It will ask for your GitHub Username and Password.

Username: Your GitHub name.

Password: PASTE YOUR TOKEN HERE. (Note: When you paste it, the screen will stay blank for security reasons. It is there, just hit Enter!)

A new folder named Industrial-Robotics-Pick-Place will appear on the left. Double-click to open it!


Step 4: Tell Git Who You Are
Type these two commands into the Command Window so I know whose code is whose:
!git config --global user.email "your_github_email@example.com"
!git config --global user.name "Your First Name"



💻 PHASE 2: The Daily Workflow (How to work on your code)
Whenever you sit down to work on your assigned task (like build_robot.m or calculate_kinematics.m), you MUST follow these 4 steps in the Command Window:

1. PULL (Get the latest updates)
Always do this before you start typing so you have my latest main.m file!
!git pull

2. CODE & SAVE
Write your math and logic inside your assigned .m file. Click the Save button (Ctrl+S / Cmd+S). Rule: DO NOT edit anyone else's file, and DO NOT edit main.m.

3. STAGE & COMMIT (Take a snapshot of your work)
When you are done for the day, tell Git to save a snapshot of your progress:
!git add .
!git commit -m "Finished writing the DH parameter table" (Change the message inside the quotes to describe what you did).

4. PUSH (Send it to the cloud)
Upload your saved snapshot to our shared GitHub so I can integrate it:
!git push
(It will ask for your Username and that long Token password again. Remember, the password stays invisible when you paste it!).


🚨 The Golden Rules
Only work in your assigned file.

Always !git pull before you start.

Always !git push when you finish.
