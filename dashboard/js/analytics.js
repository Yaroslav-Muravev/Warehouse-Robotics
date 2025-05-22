console.log('analytics.js loaded');

document.addEventListener('DOMContentLoaded', () => {
    const analyticsContent = document.createElement('div');
    analyticsContent.innerHTML = '<h1>Analytics Page</h1><p>This is where analytics data will be displayed.</p>';

    const analyticsSection = document.getElementById('analytics');
    if (analyticsSection) {
        analyticsSection.appendChild(analyticsContent);
    }
});