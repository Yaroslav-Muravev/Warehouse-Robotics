const warehouseData = {
    robots: [
        { id: 'R-101', name: 'R-101', task: 'Picking items for Order #4562', zone: 'Zone B', progress: 65, status: 'Active' },
        { id: 'R-102', name: 'R-102', task: 'Transporting to Packing Station', zone: 'Zone C', progress: 42, status: 'Active' },
        { id: 'R-103', name: 'R-103', task: 'Charging (85%)', zone: 'Charging Station', progress: 85, status: 'Charging' },
        { id: 'R-104', name: 'R-104', task: 'Maintenance Required', zone: 'Service Area', progress: 100, status: 'Error' },
        { id: 'R-105', name: 'R-105', task: 'Inventory Scanning', zone: 'Zone A', progress: 78, status: 'Active' },
    ],
    tasks: [
        { id: 'T-4562', type: 'Order Picking', robot: 'R-101', priority: 'Medium', status: 'In Progress', progress: 65 },
        { id: 'T-4563', type: 'Transport', robot: 'R-102', priority: 'Low', status: 'In Progress', progress: 42 },
        { id: 'T-4564', type: 'Inventory Scan', robot: 'R-105', priority: 'High', status: 'Pending', progress: 10 },
        { id: 'T-4565', type: 'Replenishment', robot: 'R-103', priority: 'Medium', status: 'Completed', progress: 100 },
        { id: 'T-4566', type: 'Packing', robot: 'R-102', priority: 'High', status: 'In Progress', progress: 88 },
    ],
    shelves: [
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
  ]
};

function getWarehouseData() {
    return warehouseData;
}