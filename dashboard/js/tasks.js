console.log('tasks.js loaded');

function createTaskElement(task) {
    const taskElement = document.createElement('div');
    taskElement.classList.add('task-item');

    taskElement.innerHTML = `
      <h3>${task.name}</h3>
      <p>ID: ${task.id}</p>
      <p>Robot: ${task.robot}</p>
      <p>Status: ${task.status}</p>
      <p>Progress: ${task.progress}%</p>
    `;

    return taskElement;
  }

  function renderTaskList(tasks) {
    const taskListContainer = document.getElementById('taskList');
    taskListContainer.innerHTML = '';

    tasks.forEach(task => {
      const taskElement = createTaskElement(task);
      taskListContainer.appendChild(taskElement);
    });
  }

  const mockTasks = [
    { id: 'T101', name: 'Pickup Order A', robot: 'R101', status: 'In Progress', progress: 25 },
    { id: 'T102', name: 'Deliver to Station B', robot: 'R102', status: 'Pending', progress: 0 },
    { id: 'T103', name: 'Charge Robot R103', robot: 'R103', status: 'Completed', progress: 100 },
  ];

  document.addEventListener('DOMContentLoaded', () => {
    renderTaskList(mockTasks);
  });