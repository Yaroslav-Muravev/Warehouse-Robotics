const navLinks = document.querySelectorAll('.nav-link');
const pages = document.querySelectorAll('.page');
const pageTitle = document.getElementById('pageTitle');
const newTaskBtn = document.getElementById('newTaskBtn');

navLinks.forEach(link => {
    link.addEventListener('click', function() {
        navLinks.forEach(navLink => navLink.classList.remove('active'));
        pages.forEach(page => page.classList.remove('active'));

        this.classList.add('active');
        const targetPageId = this.getAttribute('data-page');
        const targetPage = document.getElementById(targetPageId);
        targetPage.classList.add('active');

        pageTitle.textContent = this.textContent;
    });
});

newTaskBtn.addEventListener('click', () => {
    alert('New task creation functionality will be implemented here.');
});

function loadDashboardData() {
    const activeRobotsCount = document.getElementById('activeRobotsCount');
    const tasksInProgressCount = document.getElementById('tasksInProgressCount');
    const completedTodayCount = document.getElementById('completedTodayCount');
    const issuesReportedCount = document.getElementById('issuesReportedCount');

    activeRobotsCount.textContent = 12;
    tasksInProgressCount.textContent = 8;
    completedTodayCount.textContent = 24;
    issuesReportedCount.textContent = 2;
    */
}

function loadWarehouseMap() {
    const warehouseMap = document.getElementById('warehouseMap');

    const shelfData = [
        { id: 'S-101', x: 5, y: 10, label: 'A1' },
        { id: 'S-102', x: 25, y: 10, label: 'A2' },
        { id: 'S-103', x: 45, y: 10, label: 'A3' },
        { id: 'S-104', x: 65, y: 10, label: 'A4' },
        { id: 'S-105', x: 85, y: 10, label: 'A5' },
        { id: 'S-201', x: 5, y: 30, label: 'B1' },
        { id: 'S-202', x: 25, y: 30, label: 'B2' },
        { id: 'S-203', x: 45, y: 30, label: 'B3' },
        { id: 'S-204', x: 65, y: 30, label: 'B4' },
        { id: 'S-205', x: 85, y: 30, label: 'B5' },
    ];

    shelfData.forEach(shelf => {
        const shelfMarker = document.createElement('div');
        shelfMarker.classList.add('shelf-marker');
        shelfMarker.style.left = `${shelf.x}%`;
        shelfMarker.style.top = `${shelf.y}%`;
        shelfMarker.textContent = shelf.label;
        warehouseMap.appendChild(shelfMarker);
    });
}

function loadRobotListData() {
    const robotListData = [
        { id: 'R-101', name: 'R-101', task: 'Picking items for Order #4562', zone: 'Zone B', progress: 65, status: 'Active' },
        { id: 'R-102', name: 'R-102', task: 'Transporting to Packing Station', zone: 'Zone C', progress: 42, status: 'Active' },
        { id: 'R-103', name: 'R-103', task: 'Charging (85%)', zone: 'Charging Station', progress: 85, status: 'Charging' },
        { id: 'R-104', name: 'R-104', task: 'Maintenance Required', zone: 'Service Area', progress: 100, status: 'Error' },
        { id: 'R-105', name: 'R-105', task: 'Inventory Scanning', zone: 'Zone A', progress: 78, status: 'Active' },
    ];

    const robotList = document.getElementById('robotList');
    robotList.innerHTML = '<div class="divide-y divide-gray-200">';

    robotListData.forEach(robot => {
        const robotItem = document.createElement('div');
        robotItem.classList.add('p-4', 'hover:bg-gray-50', 'cursor-pointer', 'robot-item');
        robotItem.setAttribute('data-robot-id', robot.id);

        const statusColor = robot.status === 'Active' ? 'green' : robot.status === 'Charging' ? 'yellow' : 'red';
        const zoneColor = robot.zone === 'Zone A' ? 'yellow' : robot.zone === 'Zone B' ? 'blue' : robot.zone === 'Zone C' ? 'green' : 'purple';

        robotItem.innerHTML = `
            <div class="flex justify-between items-start">
                <div>
                    <div class="flex items-center">
                        <div class="w-3 h-3 rounded-full bg-${statusColor}-500 mr-2"></div>
                        <h4 class="font-medium">${robot.name}</h4>
                    </div>
                    <p class="text-sm text-gray-500 mt-1">${robot.task}</p>
                </div>
                <span class="text-xs bg-${zoneColor}-100 text-${zoneColor}-800 px-2 py-1 rounded">${robot.zone}</span>
            </div>
            <div class="mt-3">
                <div class="flex justify-between text-xs text-gray-500 mb-1">
                    <span>${robot.status === 'Charging' ? 'Battery Level' : 'Task Progress'}</span>
                    <span>${robot.progress}%</span>
                </div>
                <div class="progress-bar bg-gray-200">
                    <div class="progress-bar-fill bg-${statusColor}-500" style="width: ${robot.progress}%"></div>
                </div>
            </div>
        `;
        robotList.appendChild(robotItem);
    });
    robotList.innerHTML += '</div>';
}

function loadTaskQueueData() {
  const taskQueueData = [
    { id: 'T-4562', type: 'Order Picking', robot: 'R-101', priority: 'Medium', status: 'In Progress', progress: 65 },
    { id: 'T-4563', type: 'Transport', robot: 'R-102', priority: 'Low', status: 'In Progress', progress: 42 },
    { id: 'T-4564', type: 'Inventory Scan', robot: 'R-105', priority: 'High', status: 'Pending', progress: 10 },
    { id: 'T-4565', type: 'Replenishment', robot: 'R-103', priority: 'Medium', status: 'Completed', progress: 100 },
    { id: 'T-4566', type: 'Packing', robot: 'R-102', priority: 'High', status: 'In Progress', progress: 88 },
  ];

  const taskQueueTable = document.getElementById('taskQueue').querySelector('tbody');
  taskQueueTable.innerHTML = '';

  taskQueueData.forEach(task => {
    const row = document.createElement('tr');
    const priorityColor = task.priority === 'High' ? 'red' : task.priority === 'Medium' ? 'yellow' : 'green';
    const statusColor = task.status === 'In Progress' ? 'blue' : task.status === 'Pending' ? 'yellow' : 'green';

    row.innerHTML = `
      <td class="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">${task.id}</td>
      <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-500">${task.type}</td>
      <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-500">${task.robot}</td>
      <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
        <span class="px-2 py-1 bg-${priorityColor}-100 text-${priorityColor}-800 rounded-full text-xs">${task.priority}</span>
      </td>
      <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
        <span class="px-2 py-1 bg-${statusColor}-100 text-${statusColor}-800 rounded-full text-xs">${task.status}</span>
      </td>
      <td class="px-6 py-4 whitespace-nowrap">
        <div class="w-full bg-gray-200 rounded-full h-2">
          <div class="bg-${statusColor}-600 h-2 rounded-full" style="width: ${task.progress}%"></div>
        </div>
      </td>
    `;
    taskQueueTable.appendChild(row);
  });
}

document.addEventListener('DOMContentLoaded', () => {
    loadDashboardData();
    loadWarehouseMap();
    loadRobotListData();
    loadTaskQueueData();

    const dashboardLink = document.querySelector('[data-page="dashboard"]');
    if (dashboardLink) {
        dashboardLink.click();
    }
});