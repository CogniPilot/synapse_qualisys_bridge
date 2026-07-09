module.exports = {
  testDir: ".",
  timeout: 60_000,
  workers: 1,
  reporter: process.env.CI ? "github" : "list",
  use: {
    browserName: "chromium",
    headless: true,
    trace: "retain-on-failure",
  },
};
