const fs = require('fs');
const { ChartJSNodeCanvas } = require('chartjs-node-canvas');

// Parse log file to extract events
function parseLog(filename)
{
    const lines = fs.readFileSync(filename, 'utf8').split('\n').slice(14); // Skip the first 14 lines (header)
    const events = [];

    const timestampRegex = /\[(\d+)\]/g;

    lines.forEach(line =>
    {
        const matches = [...line.matchAll(timestampRegex)];
        if (matches.length > 0)
        {
            const timestamp = parseInt(matches[matches.length - 1][1]); // Last timestamp
            let type;
            if (line.includes('RISING')) type = 'RISING EDGE';
            else if (line.includes('FALLING')) type = 'FALLING EDGE';
            else type = 'UART';
            events.push({ timestamp, type, message: line.trim() });
        }
    });

    return events;
}

// Create a timeline visualization using Chart.js
async function createVisualization(events)
{
    const width = 1200; // Width of the image
    const height = 400; // Height of the image
    const chartJSNodeCanvas = new ChartJSNodeCanvas({ width, height });

    // Prepare data
    const startTime = Math.min(...events.map(e => e.timestamp));
    const endTime = Math.max(...events.map(e => e.timestamp));

    const uartEvents = events.filter(e => e.type === 'UART');
    const risingEdgeEvents = events.filter(e => e.type === 'RISING EDGE');
    const fallingEdgeEvents = events.filter(e => e.type === 'FALLING EDGE');

    const datasets = [];

    // UART Events Dataset
    if (uartEvents.length > 0)
    {
        datasets.push({
            label: 'UART Events',
            data: uartEvents.map(e => ({ x: (e.timestamp - startTime) / 1000, y: 1 })),
            backgroundColor: 'blue',
            pointRadius: 5,
            showLine: false,
        });
    }

    // RISING EDGE Events Dataset
    if (risingEdgeEvents.length > 0)
    {
        datasets.push({
            label: 'RISING EDGE',
            data: risingEdgeEvents.map(e => ({ x: (e.timestamp - startTime) / 1000, y: 2 })),
            backgroundColor: 'red',
            pointRadius: 5,
            showLine: false,
        });
    }

    // FALLING EDGE Events Dataset
    if (fallingEdgeEvents.length > 0)
    {
        datasets.push({
            label: 'FALLING EDGE',
            data: fallingEdgeEvents.map(e => ({ x: (e.timestamp - startTime) / 1000, y: 3 })),
            backgroundColor: 'green',
            pointRadius: 5,
            showLine: false,
        });
    }

    // Chart configuration
    const configuration = {
        type: 'scatter',
        data: {
            datasets: datasets,
        },
        options: {
            responsive: false,
            plugins: {
                legend: {
                    position: 'top',
                },
                title: {
                    display: true,
                    text: 'Event Timeline',
                },
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'Time (s)',
                    },
                    ticks: {
                        callback: value => `${value.toFixed(1)} s`, // Format ticks as seconds with one decimal
                    },
                },
                y: {
                    display: false,
                    min: 0,
                    max: 4,
                },
            },
        },
    };

    // Render the chart and return the image buffer
    const imageBuffer = await chartJSNodeCanvas.renderToBuffer(configuration);
    return imageBuffer;
}

// Main execution
(async () =>
{
    const args = process.argv.slice(2); // Get command-line arguments
    const logFilePath = args[0] || 'log.txt'; // Use provided path or default to 'log.txt'

    if (!fs.existsSync(logFilePath))
    {
        console.error(`Log file not found: ${logFilePath}`);
        process.exit(1);
    }

    const events = parseLog(logFilePath); // Load and parse the log file
    const image = await createVisualization(events); // Generate the visualization
    fs.writeFileSync('plot.png', image); // Save the chart as an image
    console.log('The plot has been successfully created and saved as "plot.png".');
})();
