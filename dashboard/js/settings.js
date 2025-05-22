console.log('settings.js loaded');

document.addEventListener('DOMContentLoaded', () => {
    const settingsContent = document.createElement('div');
    settingsContent.innerHTML = '<h1>Settings Page</h1><p>This is where user settings will be managed.</p>';

    const settingsSection = document.getElementById('settings');
    if (settingsSection) {
        settingsSection.appendChild(settingsContent);
    }
});