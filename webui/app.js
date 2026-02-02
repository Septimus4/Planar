/**
 * Planar Web UI - Application Controller
 * 
 * Connects to capture daemon on Raspberry Pi via WebSocket
 * and provides real-time LiDAR visualization and session control.
 */

class PlanarApp {
    constructor() {
        // Connection state
        this.ws = null;
        this.apiBase = '';
        this.connected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 2000;
        
        // Session state
        this.sessionActive = false;
        this.sessionId = null;
        this.stationCount = 0;
        
        // Canvas state
        this.canvas = null;
        this.ctx = null;
        this.points = [];
        this.frameCount = 0;
        this.lastFpsTime = Date.now();
        this.currentFps = 0;
        
        // Level indicator
        this.levelCanvas = null;
        this.levelCtx = null;
        
        // Options
        this.autoRotate = true;
        this.showGrid = true;
        this.colorScheme = 'quality';
        this.viewRotation = 0;
        this.scale = 50; // pixels per meter
        
        // IMU data
        this.imuData = {
            gyroZ: 0,
            accelZ: 0,
            accelX: 0,
            accelY: 0,
            temperature: 0
        };
        
        this.init();
    }
    
    init() {
        this.setupCanvas();
        this.setupLevelIndicator();
        this.setupEventListeners();
        this.startRenderLoop();
        this.log('info', 'Planar UI initialized');
    }
    
    // ==================== Canvas Setup ====================
    
    setupCanvas() {
        this.canvas = document.getElementById('previewCanvas');
        this.ctx = this.canvas.getContext('2d');
        
        // Handle high DPI displays
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
        
        // Mouse wheel zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.scale *= zoomFactor;
            this.scale = Math.max(10, Math.min(200, this.scale));
        });
    }
    
    resizeCanvas() {
        const container = this.canvas.parentElement;
        const rect = container.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        
        this.ctx.scale(dpr, dpr);
        this.canvasWidth = rect.width;
        this.canvasHeight = rect.height;
    }
    
    setupLevelIndicator() {
        this.levelCanvas = document.getElementById('levelCanvas');
        this.levelCtx = this.levelCanvas.getContext('2d');
    }
    
    // ==================== Event Listeners ====================
    
    setupEventListeners() {
        // Connection
        document.getElementById('connectBtn').addEventListener('click', () => this.toggleConnection());
        document.getElementById('hostInput').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') this.toggleConnection();
        });
        
        // Device controls
        document.getElementById('lidarStartBtn').addEventListener('click', () => this.controlLidar('start'));
        document.getElementById('lidarStopBtn').addEventListener('click', () => this.controlLidar('stop'));
        document.getElementById('imuStartBtn').addEventListener('click', () => this.controlImu('start'));
        document.getElementById('imuStopBtn').addEventListener('click', () => this.controlImu('stop'));
        document.getElementById('imuSelfTestBtn').addEventListener('click', () => this.imuSelfTest());
        
        // Session controls
        document.getElementById('sessionStartBtn').addEventListener('click', () => this.startSession());
        document.getElementById('markStationBtn').addEventListener('click', () => this.markStation());
        document.getElementById('sessionStopBtn').addEventListener('click', () => this.stopSession());
        
        // Preview options
        document.getElementById('autoRotate').addEventListener('change', (e) => {
            this.autoRotate = e.target.checked;
        });
        document.getElementById('showGrid').addEventListener('change', (e) => {
            this.showGrid = e.target.checked;
        });
        document.getElementById('colorScheme').addEventListener('change', (e) => {
            this.colorScheme = e.target.value;
        });
        
        // Log
        document.getElementById('clearLogBtn').addEventListener('click', () => this.clearLog());
    }
    
    // ==================== Connection Management ====================
    
    toggleConnection() {
        if (this.connected) {
            this.disconnect();
        } else {
            this.connect();
        }
    }
    
    connect() {
        const host = document.getElementById('hostInput').value.trim();
        if (!host) {
            this.toast('error', 'Please enter a host address');
            return;
        }
        
        this.apiBase = `http://${host}`;
        const wsUrl = `ws://${host}/ws`;
        
        this.updateConnectionStatus('connecting', 'Connecting...');
        this.log('info', `Connecting to ${host}...`);
        
        try {
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => {
                this.connected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionStatus('connected', 'Connected');
                document.getElementById('canvasOverlay').classList.add('hidden');
                document.getElementById('connectBtn').textContent = 'Disconnect';
                this.log('success', 'Connected to Planar daemon');
                this.toast('success', 'Connected to Raspberry Pi');
                this.fetchStatus();
            };
            
            this.ws.onmessage = (event) => {
                this.handleMessage(JSON.parse(event.data));
            };
            
            this.ws.onclose = () => {
                this.handleDisconnect();
            };
            
            this.ws.onerror = (error) => {
                this.log('error', 'WebSocket error');
                console.error('WebSocket error:', error);
            };
            
        } catch (error) {
            this.log('error', `Connection failed: ${error.message}`);
            this.toast('error', 'Connection failed');
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
    
    handleDisconnect() {
        this.connected = false;
        this.updateConnectionStatus('disconnected', 'Disconnected');
        document.getElementById('canvasOverlay').classList.remove('hidden');
        document.getElementById('connectBtn').textContent = 'Connect';
        this.points = [];
        
        // Auto-reconnect
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            this.log('warning', `Connection lost. Reconnecting (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);
            setTimeout(() => {
                if (!this.connected) this.connect();
            }, this.reconnectDelay);
        } else {
            this.log('error', 'Max reconnection attempts reached');
            this.toast('error', 'Connection lost');
        }
    }
    
    updateConnectionStatus(status, text) {
        const statusEl = document.getElementById('connectionStatus');
        const dot = statusEl.querySelector('.status-dot');
        const textEl = statusEl.querySelector('.status-text');
        
        dot.className = `status-dot ${status}`;
        textEl.textContent = text;
    }
    
    // ==================== Message Handling ====================
    
    handleMessage(msg) {
        switch (msg.type) {
            case 'lidar_frame':
                this.handleLidarFrame(msg);
                break;
            case 'imu_sample':
                this.handleImuSample(msg);
                break;
            case 'status':
                this.updateStatus(msg);
                break;
            case 'event':
                this.handleEvent(msg);
                break;
            default:
                console.log('Unknown message type:', msg.type);
        }
    }
    
    handleLidarFrame(msg) {
        // Points: [[angle_deg, distance_m, quality], ...]
        this.points = msg.points || [];
        this.frameCount++;
        
        // Update stats
        document.getElementById('pointCount').textContent = this.points.length;
        
        // Calculate FPS
        const now = Date.now();
        if (now - this.lastFpsTime >= 1000) {
            this.currentFps = this.frameCount;
            this.frameCount = 0;
            this.lastFpsTime = now;
            document.getElementById('fps').textContent = this.currentFps;
        }
        
        // Update range
        if (this.points.length > 0) {
            const distances = this.points.map(p => p[1]).filter(d => d > 0);
            const maxDist = Math.max(...distances);
            document.getElementById('rangeDisplay').textContent = maxDist.toFixed(2) + 'm';
        }
    }
    
    handleImuSample(msg) {
        this.imuData = {
            gyroZ: msg.gyro_z || 0,
            accelZ: msg.accel_z || 0,
            accelX: msg.accel_x || 0,
            accelY: msg.accel_y || 0,
            temperature: msg.temperature || 0
        };
        
        document.getElementById('gyroZ').textContent = this.imuData.gyroZ.toFixed(2) + ' °/s';
        document.getElementById('accelZ').textContent = this.imuData.accelZ.toFixed(2) + ' m/s²';
        document.getElementById('temperature').textContent = this.imuData.temperature.toFixed(1) + ' °C';
        
        // Level status
        const tilt = Math.sqrt(this.imuData.accelX ** 2 + this.imuData.accelY ** 2);
        const levelEl = document.getElementById('levelStatus');
        if (tilt < 0.2) {
            levelEl.textContent = '✓ Level';
            levelEl.className = 'imu-value ok';
        } else if (tilt < 0.5) {
            levelEl.textContent = '⚠ Slight tilt';
            levelEl.className = 'imu-value warning';
        } else {
            levelEl.textContent = '✗ Not level';
            levelEl.className = 'imu-value error';
        }
        
        this.drawLevelIndicator();
    }
    
    handleEvent(msg) {
        const eventType = msg.event || msg.name;
        this.log('info', `Event: ${eventType}`);
        
        if (eventType === 'session_start') {
            this.sessionActive = true;
            this.sessionId = msg.session_id;
            this.stationCount = 0;
            this.updateSessionUI();
        } else if (eventType === 'station_captured') {
            this.stationCount++;
            this.updateSessionUI();
            this.toast('success', `Station ${this.stationCount} captured!`);
        } else if (eventType === 'session_stop') {
            this.sessionActive = false;
            this.updateSessionUI();
            this.toast('success', 'Session completed');
        }
    }
    
    updateStatus(msg) {
        // LiDAR status
        const lidarEl = document.getElementById('lidarStatus');
        if (msg.lidar?.running) {
            lidarEl.textContent = 'Running';
            lidarEl.className = 'status-indicator ok';
        } else if (msg.lidar?.connected) {
            lidarEl.textContent = 'Stopped';
            lidarEl.className = 'status-indicator warning';
        } else {
            lidarEl.textContent = 'Disconnected';
            lidarEl.className = 'status-indicator error';
        }
        
        // IMU status
        const imuEl = document.getElementById('imuStatus');
        if (msg.imu?.running) {
            imuEl.textContent = 'Running';
            imuEl.className = 'status-indicator ok';
        } else if (msg.imu?.connected) {
            imuEl.textContent = 'Stopped';
            imuEl.className = 'status-indicator warning';
        } else {
            imuEl.textContent = 'Disconnected';
            imuEl.className = 'status-indicator error';
        }
        
        // Session status
        const sessionEl = document.getElementById('sessionStatus');
        if (msg.session?.active) {
            sessionEl.textContent = 'Active';
            sessionEl.className = 'status-indicator ok';
            this.sessionActive = true;
            this.sessionId = msg.session.id;
            this.stationCount = msg.session.stations || 0;
        } else {
            sessionEl.textContent = 'Inactive';
            sessionEl.className = 'status-indicator';
            this.sessionActive = false;
        }
        
        document.getElementById('stationCount').textContent = this.stationCount;
        this.updateSessionUI();
    }
    
    updateSessionUI() {
        document.getElementById('sessionStartBtn').disabled = this.sessionActive;
        document.getElementById('markStationBtn').disabled = !this.sessionActive;
        document.getElementById('sessionStopBtn').disabled = !this.sessionActive;
        document.getElementById('stationCount').textContent = this.stationCount;
    }
    
    // ==================== API Calls ====================
    
    async fetchStatus() {
        try {
            const response = await fetch(`${this.apiBase}/status`);
            const data = await response.json();
            this.updateStatus(data);
        } catch (error) {
            this.log('error', 'Failed to fetch status');
        }
    }
    
    async controlLidar(action) {
        try {
            const response = await fetch(`${this.apiBase}/lidar/${action}`, { method: 'POST' });
            const data = await response.json();
            if (data.status === 'ok') {
                this.log('success', `LiDAR ${action}ed`);
                this.fetchStatus();
            } else {
                this.log('error', `LiDAR ${action} failed: ${data.error}`);
            }
        } catch (error) {
            this.log('error', `LiDAR ${action} failed: ${error.message}`);
        }
    }
    
    async controlImu(action) {
        try {
            const response = await fetch(`${this.apiBase}/imu/${action}`, { method: 'POST' });
            const data = await response.json();
            if (data.status === 'ok') {
                this.log('success', `IMU ${action}ed`);
                this.fetchStatus();
            } else {
                this.log('error', `IMU ${action} failed: ${data.error}`);
            }
        } catch (error) {
            this.log('error', `IMU ${action} failed: ${error.message}`);
        }
    }
    
    async imuSelfTest() {
        try {
            const response = await fetch(`${this.apiBase}/imu/self_test`, { method: 'POST' });
            const data = await response.json();
            if (data.passed) {
                this.log('success', 'IMU self-test passed');
                this.toast('success', 'IMU self-test passed');
            } else {
                this.log('error', `IMU self-test failed: ${data.message}`);
                this.toast('error', 'IMU self-test failed');
            }
        } catch (error) {
            this.log('error', `IMU self-test failed: ${error.message}`);
        }
    }
    
    async startSession() {
        const name = document.getElementById('sessionName').value.trim();
        const sessionName = name || `scan_${Date.now()}`;
        
        try {
            const response = await fetch(`${this.apiBase}/session/start`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name: sessionName })
            });
            const data = await response.json();
            if (data.status === 'ok') {
                this.sessionActive = true;
                this.sessionId = data.session_id;
                this.stationCount = 0;
                this.updateSessionUI();
                this.log('success', `Session started: ${sessionName}`);
                this.toast('success', 'Session started');
            } else {
                this.log('error', `Start session failed: ${data.error}`);
            }
        } catch (error) {
            this.log('error', `Start session failed: ${error.message}`);
        }
    }
    
    async markStation() {
        try {
            const response = await fetch(`${this.apiBase}/session/mark_station`, { method: 'POST' });
            const data = await response.json();
            if (data.status === 'ok') {
                this.log('success', `Station marked (${data.station_number})`);
            } else {
                this.log('error', `Mark station failed: ${data.error}`);
            }
        } catch (error) {
            this.log('error', `Mark station failed: ${error.message}`);
        }
    }
    
    async stopSession() {
        try {
            const response = await fetch(`${this.apiBase}/session/stop`, { method: 'POST' });
            const data = await response.json();
            if (data.status === 'ok') {
                this.sessionActive = false;
                this.updateSessionUI();
                this.log('success', 'Session stopped');
                this.toast('success', `Session complete (${this.stationCount} stations)`);
            } else {
                this.log('error', `Stop session failed: ${data.error}`);
            }
        } catch (error) {
            this.log('error', `Stop session failed: ${error.message}`);
        }
    }
    
    // ==================== Rendering ====================
    
    startRenderLoop() {
        const render = () => {
            this.drawLidar();
            requestAnimationFrame(render);
        };
        render();
    }
    
    drawLidar() {
        const ctx = this.ctx;
        const w = this.canvasWidth;
        const h = this.canvasHeight;
        const cx = w / 2;
        const cy = h / 2;
        
        // Clear canvas
        ctx.fillStyle = '#010409';
        ctx.fillRect(0, 0, w, h);
        
        // Draw grid
        if (this.showGrid) {
            this.drawGrid(cx, cy);
        }
        
        // Auto-rotate view
        if (this.autoRotate && this.points.length > 0) {
            this.viewRotation = (this.viewRotation + 0.5) % 360;
        }
        
        // Draw points
        for (const point of this.points) {
            const [angleDeg, distanceM, quality] = point;
            if (distanceM <= 0) continue;
            
            const angle = (angleDeg + this.viewRotation) * Math.PI / 180;
            const r = distanceM * this.scale;
            
            const x = cx + r * Math.cos(angle);
            const y = cy - r * Math.sin(angle);
            
            // Color by scheme
            let color;
            switch (this.colorScheme) {
                case 'quality':
                    color = this.qualityToColor(quality);
                    break;
                case 'distance':
                    color = this.distanceToColor(distanceM);
                    break;
                default:
                    color = '#ffffff';
            }
            
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(x, y, 2, 0, Math.PI * 2);
            ctx.fill();
        }
        
        // Draw origin marker
        ctx.fillStyle = '#58a6ff';
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI * 2);
        ctx.fill();
        
        // Draw orientation indicator
        ctx.strokeStyle = '#58a6ff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        const indicatorAngle = this.viewRotation * Math.PI / 180;
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + 30 * Math.cos(indicatorAngle), cy - 30 * Math.sin(indicatorAngle));
        ctx.stroke();
    }
    
    drawGrid(cx, cy) {
        const ctx = this.ctx;
        
        ctx.strokeStyle = '#21262d';
        ctx.lineWidth = 1;
        
        // Radial lines
        for (let a = 0; a < 360; a += 45) {
            const angle = a * Math.PI / 180;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(
                cx + 1000 * Math.cos(angle),
                cy - 1000 * Math.sin(angle)
            );
            ctx.stroke();
        }
        
        // Concentric circles (every meter)
        for (let r = 1; r <= 20; r++) {
            const radius = r * this.scale;
            ctx.beginPath();
            ctx.arc(cx, cy, radius, 0, Math.PI * 2);
            ctx.stroke();
            
            // Label
            if (r <= 10) {
                ctx.fillStyle = '#6e7681';
                ctx.font = '10px sans-serif';
                ctx.fillText(`${r}m`, cx + radius + 5, cy);
            }
        }
    }
    
    qualityToColor(quality) {
        // Quality 0-100 -> red to green
        const normalized = Math.min(100, Math.max(0, quality)) / 100;
        const r = Math.floor((1 - normalized) * 255);
        const g = Math.floor(normalized * 255);
        return `rgb(${r}, ${g}, 100)`;
    }
    
    distanceToColor(distance) {
        // Distance 0-10m -> blue to red
        const normalized = Math.min(10, Math.max(0, distance)) / 10;
        const r = Math.floor(normalized * 255);
        const b = Math.floor((1 - normalized) * 255);
        return `rgb(${r}, 100, ${b})`;
    }
    
    drawLevelIndicator() {
        const ctx = this.levelCtx;
        const size = 100;
        const cx = size / 2;
        const cy = size / 2;
        
        // Clear
        ctx.fillStyle = '#21262d';
        ctx.fillRect(0, 0, size, size);
        
        // Draw crosshairs
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(cx, 10);
        ctx.lineTo(cx, size - 10);
        ctx.moveTo(10, cy);
        ctx.lineTo(size - 10, cy);
        ctx.stroke();
        
        // Draw target circle
        ctx.strokeStyle = '#3fb950';
        ctx.beginPath();
        ctx.arc(cx, cy, 15, 0, Math.PI * 2);
        ctx.stroke();
        
        // Draw bubble (accel X/Y indicate tilt)
        const bubbleX = cx + (this.imuData.accelX / 9.8) * 30;
        const bubbleY = cy - (this.imuData.accelY / 9.8) * 30;
        
        const tilt = Math.sqrt(this.imuData.accelX ** 2 + this.imuData.accelY ** 2);
        ctx.fillStyle = tilt < 0.2 ? '#3fb950' : tilt < 0.5 ? '#d29922' : '#f85149';
        ctx.beginPath();
        ctx.arc(bubbleX, bubbleY, 8, 0, Math.PI * 2);
        ctx.fill();
    }
    
    // ==================== Logging ====================
    
    log(level, message) {
        const logContent = document.getElementById('logContent');
        const time = new Date().toLocaleTimeString();
        
        const entry = document.createElement('div');
        entry.className = `log-entry ${level}`;
        entry.innerHTML = `<span class="log-time">${time}</span><span class="log-message">${message}</span>`;
        
        logContent.appendChild(entry);
        logContent.scrollTop = logContent.scrollHeight;
        
        // Limit log entries
        while (logContent.children.length > 100) {
            logContent.removeChild(logContent.firstChild);
        }
    }
    
    clearLog() {
        document.getElementById('logContent').innerHTML = '';
        this.log('info', 'Log cleared');
    }
    
    toast(type, message) {
        const container = document.getElementById('toastContainer');
        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        toast.textContent = message;
        
        container.appendChild(toast);
        
        setTimeout(() => {
            toast.style.opacity = '0';
            setTimeout(() => toast.remove(), 300);
        }, 3000);
    }
}

// Initialize app when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.planarApp = new PlanarApp();
});
