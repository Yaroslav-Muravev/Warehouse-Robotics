// Этот файл может быть расширен для обработки интерактивности, специфичной для панели мониторинга
// Например, обновление данных в реальном времени, обработка взаимодействий с картой и т.д.

console.log('dashboard.js loaded');

// Пример функции для обновления количества активных роботов (можно вызывать через setInterval)
function updateActiveRobotsCount(newCount) {
    const activeRobotsCountElement = document.getElementById('activeRobotsCount');
    if (activeRobotsCountElement) {
        activeRobotsCountElement.textContent = newCount;
    }
}

// Пример функции для добавления робота на карту
function addRobotToMap(robotId, x, y, zone) {
  const warehouseMap = document.getElementById('warehouseMap');
  if (!warehouseMap) return;

  const robotMarker = document.createElement('div');
  robotMarker.classList.add('robot-marker');
  robotMarker.textContent = robotId;
  robotMarker.style.left = `${x}%`;
  robotMarker.style.top = `${y}%`;

  // Добавляем класс в зависимости от зоны
  let zoneColorClass = '';
  switch (zone) {
    case 'Zone A':
      zoneColorClass = 'bg-yellow-500';
      break;
    case 'Zone B':
      zoneColorClass = 'bg-blue-500';
      break;
    case 'Zone C':
      zoneColorClass = 'bg-green-500';
      break;
    case 'Charging Station':
      zoneColorClass = 'bg-purple-500';
      break;
    case 'Service Area':
        zoneColorClass = 'bg-red-500';
        break;
    default:
      zoneColorClass = 'bg-gray-500';
  }
  robotMarker.classList.add(zoneColorClass);
  warehouseMap.appendChild(robotMarker);
}