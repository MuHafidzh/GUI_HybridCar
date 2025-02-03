  <template>
    <div class="bg-black min-h-screen p-4 flex flex-col items-center">
      <!-- Header untuk Logo dan IP/RTC -->
      <div class="header flex justify-between items-center w-full max-w-4xl mb-4 bg-gray-800 p-4 rounded-xl">
        <div class="logo-container flex items-center">
          <img src="../assets/icons/kedaireka.jpeg" alt="kedaireka" class="logo">
         <!-- <img src="../assets/icons/mri.png" alt="MRI Logo" class="logo ml-2"> -->
        </div>
        <div class="flex flex-col items-center">
          <span class="text-blue-400 text-3xl font-semibold">HYBRID CAR</span>
        </div>
        <div class="ip-rtc flex flex-col items-end">
          <span class="text-white font-semibold text-sm">IP: 192.168.1.1</span>
          <span class="text-white font-semibold text-sm">{{ new Date().toLocaleDateString('id-ID', { weekday: 'long' }) }}</span>
          <span class="text-white font-semibold text-sm">🕒 RTC: {{ currentTime }}</span>
        </div>
      </div>

      <!-- Monitor Data -->
      <div class="monitor-data grid grid-cols-3 gap-4 w-full max-w-4xl">
        <!-- Kolom Kiri -->
        <div class="left-col flex flex-col gap-4">
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center justify-center">
            <vue-ui-wheel :dataset="rotorSpeedDataset" :config="wheelConfig" class="text-blue-400 font-semibold"></vue-ui-wheel>
          </div>
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex items-center">
            <span class="mr-2"></span> <span class="text-bus-voltage font-semibold">HALTED</span>
          </div>
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-start justify-center">
            <!-- <vue-ui-kpi :dataset="busVoltageDataset" :config="kpiConfig" class="text-blue-400 font-semibold medium-kpi"></vue-ui-kpi> -->
            <div class="flex flex-col w-full">
              <vue-ui-kpi :dataset="busVoltageDataset" :config="kpiConfig" class="text-blue-400 font-semibold medium-kpi mb-2"></vue-ui-kpi>
            <div class="flex justify-between">
              <span class="text-sm">Throttle:</span>
              <span class="text-sm">{{ throttle }}</span>
            </div>
          </div>
        </div>
      </div>

        <!-- Kolom Tengah -->
        <div class="center-col flex flex-col gap-4">
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center justify-center">
            <vue-ui-gauge :dataset="speedDataset" :config="gaugeConfig" class="text-blue-400 font-semibold"></vue-ui-gauge>
          </div>
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center justify-center">
            <vue-ui-sparkgauge :dataset="busCurrentDataset" :config="sparkgaugeConfig" class="text-blue-400 font-semibold smaller-sparkgauge"></vue-ui-sparkgauge>
          </div>
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center justify-center">
            <vue-ui-sparkgauge :dataset="phaseCurrentDataset" :config="sparkgaugeConfig" class="text-blue-400 font-semibold smaller-sparkgauge"></vue-ui-sparkgauge>
          </div>
        </div>

        <!-- Kolom Kanan -->
        <div class="right-col flex flex-col gap-4">
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center">
            <vue-ui-thermometer :dataset="controllerTemperatureDataset" :config="controllerThermometerConfig" class="text-blue-400 font-semibold small-thermometer"></vue-ui-thermometer>
          </div>
          <div class="bg-gray-800 rounded-xl shadow-lg p-2 text-white font-semibold text-sm flex flex-col items-center">
            <vue-ui-thermometer :dataset="motorTemperatureDataset" :config="motorThermometerConfig" class="text-blue-400 font-semibold small-thermometer"></vue-ui-thermometer>
          </div>
        </div>
      </div>
    </div>
  </template>

  <script>

  import { useMesinStore } from "../stores/store.js";
  import { computed } from "vue";

  export default {
    name: "MonitoringUI",
    setup() {
    const MESIN_STATE = useMesinStore();
    const busVoltageDataset = computed(() => MESIN_STATE.dataMesin.busVoltage);
    return {
      busVoltageDataset,
    };
  },
    data() {
      return {
        currentTime: new Date().toLocaleTimeString(),
        ipAddress: "192.168.1.1", // Example IP address
        speedDataset: {
          base: 0,
          value: 60,
          series: [
            { from: 0, to: 50, color: "#ff6400" },
            { from: 50, to: 100, color: "#5f8bee" },
            { from: 100, to: 200, color: "#42d392" },
          ],
        },
        rotorSpeedDataset: {
          percentage: 75,
        },
        // busVoltageDataset: 50.4,
        busCurrentDataset: {
          value: 50,
          min: 0,
          max: 100,
          title: "Bus Current",
        },
        phaseCurrentDataset: {
          value: 90,
          min: 0,
          max: 100,
          title: "Phase Current",
        },
        controllerTemperatureDataset: {
          value: 20,
          from: -50,
          to: 150,
          steps: 20,
          colors: {
            from: "#dc3912",
            to: "#3366cc",
          }
        },
        motorTemperatureDataset: {
          value: 36,
          from: -50,
          to: 150,
          steps: 20,
          colors: {
            from: "#dc3912",
            to: "#3366cc",
          }
        },
        gaugeConfig: {
          responsive: false,
          style: {
            fontFamily: "inherit",
            chart: {
              backgroundColor: "#1A1A1A",
              color: "#CCCCCC",
              animation: {
                use: true,
                speed: 0.5,
                acceleration: 1,
              },
              title: {
                text: "Speedometer",
                color: "#FAFAFA",
                fontSize: 20,
                bold: true,
                textAlign: "center",
                paddingLeft: 0,
                paddingRight: 0,
                subtitle: {
                  color: "#A1A1A1",
                  text: "km/h",
                  fontSize: 16,
                  bold: false,
                },
              },
            },
          },
          userOptions: {
            show: false,
          },
          translations: {
            kmh: "km/h",
          },
        },
        wheelConfig: {
          responsive: false,
          style: {
            fontFamily: "inherit",
            chart: {
              backgroundColor: "#1A1A1A",
              color: "#CCCCCC",
              animation: {
                use: true,
                speed: 0.5,
                acceleration: 1,
              },
              layout: {
                wheel: {
                  ticks: {
                    rounded: true,
                    inactiveColor: "#3A3A3A",
                    activeColor: "#5f8bee",
                    sizeRatio: 0.9,
                    gradient: {
                      show: true,
                      shiftHueIntensity: 100,
                    },
                  },
                },
                innerCircle: {
                  show: true,
                  stroke: "#3A3A3A",
                  strokeWidth: 1,
                },
                percentage: {
                  show: true,
                  fontSize: 48,
                  rounding: 1,
                  bold: true,
                },
              },
              title: {
                text: "Rotor Speed",
                color: "#FAFAFA",
                fontSize: 20,
                bold: true,
                textAlign: "center",
                paddingLeft: 0,
                paddingRight: 0,
                subtitle: {
                  color: "#A1A1A1",
                  text: "rpm",
                  fontSize: 16,
                  bold: false,
                },
              },
            },
          },
          userOptions: {
            show: false,
          },
        },
        kpiConfig: {
          animationFrames: 60,
          animationValueStart: 0,
          backgroundColor: "#1A1A1A",
          fontFamily: "inherit",
          layoutClass: "p-2 rounded-md shadow-md mb-2",
          prefix: "",
          suffix: " V",
          title: "Bus Voltage",
          titleBold: true,
          titleColor: "#CCCCCC",
          titleClass: "",
          titleCss: "",
          titleFontSize: 32,
          useAnimation: true,
          valueBold: true,
          valueColor: "#6376DD",
          valueClass: "tabular-nums",
          valueCss: "",
          valueFontSize: 22,
          valueRounding: 1,
          analogDigits: {
            show: false,
            height: 20,
            color: "#6376DD",
            skeletonColor: "#2A2A2A",
          },
        },
        sparkgaugeConfig: {
          style: {
            fontFamily: "inherit",
            background: "#1A1A1A",
            height: 30,
            basePosition: 45,
            animation: {
              show: true,
              speedMs: 150,
            },
            title: {
              show: true,
              fontSize: 22,
              position: "bottom",
              textAlign: "center",
              bold: false,
              color: "#CCCCCC",
            },
            dataLabel: {
              fontSize: 12,
              autoColor: true,
              color: "#CCCCCC",
              offsetY: 0,
              bold: true,
              rounding: 0,
              prefix: "",
              suffix: " A",
            },
            colors: {
              min: "#FF0000",
              max: "#00FF00",
              showGradient: true,
            },
            track: {
              autoColor: true,
              color: "#5f8bee",
              strokeLinecap: "round",
            },
            gutter: {
              color: "#3A3A3A",
              strokeLinecap: "round",
            },
          },
        },
        controllerThermometerConfig: {
          style: {
            fontFamily: "inherit",
            chart: {
              backgroundColor: "#1A1A1A",
              color: "#CCCCCC",
              height: 240,
              thermometer: {
                width: 30,
              },
              padding: {
                top: 12,
                bottom: 12,
                left: 64,
                right: 64,
              },
              graduations: {
                show: true,
                sides: "both",
                height: 2,
                stroke: "#e1e5e8",
                strokeWidth: 1,
                showIntermediate: true,
                gradient: {
                  show: true,
                  intensity: 40,
                },
              },
              animation: {
                use: true,
                speedMs: 1000,
              },
              label: {
                fontSize: 20,
                rounding: 1,
                bold: true,
                color: "#CCCCCC",
                prefix: "",
                suffix: "",
              },
            },
            title: {
              text: "Controller Temp",
              color: "#FAFAFA",
              fontSize: 20,
              bold: true,
              textAlign: "center",
              paddingLeft: 0,
              paddingRight: 0,
              subtitle: {
                color: "#A1A1A1",
                text: "°C",
                fontSize: 16,
                bold: false,
              },
            },
          },
          userOptions: {
            show: false,
          },
        },
        motorThermometerConfig: {
          style: {
            fontFamily: "inherit",
            chart: {
              backgroundColor: "#1A1A1A",
              color: "#CCCCCC",
              height: 200,
              thermometer: {
                width: 30,
              },
              padding: {
                top: 12,
                bottom: 12,
                left: 64,
                right: 64,
              },
              graduations: {
                show: true,
                sides: "both",
                height: 2,
                stroke: "#e1e5e8",
                strokeWidth: 1,
                showIntermediate: true,
                gradient: {
                  show: true,
                  intensity: 40,
                },
              },
              animation: {
                use: true,
                speedMs: 1000,
              },
              label: {
                fontSize: 20,
                rounding: 1,
                bold: true,
                color: "#CCCCCC",
                prefix: "",
                suffix: "",
              },
            },
            title: {
              text: "Motor Temp",
              color: "#FAFAFA",
              fontSize: 20,
              bold: true,
              textAlign: "center",
              paddingLeft: 0,
              paddingRight: 0,
              subtitle: {
                color: "#A1A1A1",
                text: "°C",
                fontSize: 16,
                bold: false,
              },
            },
          },
          userOptions: {
            show: false,
          },
        },
      };
    },
    mounted() {
      this.updateTime();
    },
    methods: {
      updateTime() {
        setInterval(() => {
          this.currentTime = new Date().toLocaleTimeString();
        }, 1000);
      },
    },
  };
  </script>

  <style scoped>
  @import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');

  .bg-black {
    background-color: #000000;
  }

  .bg-gray-800 {
    background-color: #1A1A1A;
  }

  .min-h-screen {
    min-height: 100vh;
  }

  .p-4 {
    padding: 1rem;
  }

  .flex {
    display: flex;
  }

  .flex-col {
    flex-direction: column;
  }

  .items-center {
    align-items: center;
  }

  .items-end {
    align-items: flex-end;
  }

  .text-blue-400 {
    color: #63b3ed;
  }

  .text-white {
    color: #ffffff;
  }

  .text-bus-voltage {
    color: #6376DD; /* Warna yang sama dengan dataset dari bus voltage */
  }

  .font-semibold {
    font-weight: 600;
  }

  .font-bold {
    font-weight: 700;
  }

  .text-sm {
    font-size: 0.875rem;
  }

  .text-2xl {
    font-size: 1.5rem;
  }

  .text-3xl {
    font-size: 2rem;
  }

  .w-full {
    width: 100%;
  }

  .max-w-4xl {
    max-width: 64rem;
  }

  .mb-2 {
    margin-bottom: 0.5rem;
  }

  .mt-2 {
    margin-top: 0.5rem;
  }

  .mr-2 {
    margin-right: 10rem;
  }

  .grid {
    display: grid;
  }

  .grid-cols-3 {
    grid-template-columns: 2fr 2fr 1fr; /* Adjust the proportions here */
  }

  .gap-2 {
    gap: 0.5rem;
  }

  .gap-4 {
    gap: 1rem;
  }

  .text-gray-800 {
    color: #2d3748;
  }

  .space-y-2 > * + * {
    margin-top: 0.5rem;
  }

  .justify-between {
    justify-content: space-between;
  }

  .text-gray-600 {
    color: #718096;
  }

  .justify-around {
    justify-content: space-around;
  }

  .ml-2 {
    margin-left: 0.5rem;
  }

  .logo-container {
    display: flex;
    align-items: center;
  }

  .logo {
    width: 120px;
    height: auto;
  }

  .data-item, .ui-item {
    display: flex;
    align-items: center;
    font-size: 0.875rem;
  }

  .left-col, .center-col, .right-col {
    display: flex;
    flex-direction: column;
    justify-content: space-between; /* Ensure columns are evenly spaced */
  }

  .small-thermometer {
    transform: scale(0.75); /* Mengecilkan ukuran thermometer */
  }

  .medium-kpi {
    transform: scale(0.85); /* Memperbesar sedikit ukuran KPI */
  }

  .smaller-sparkgauge {
    transform: scale(0.5); /* Mengecilkan lagi ukuran Spark Gauge */
  }

  /* Media Queries for Responsive Design */
  @media (max-width: 1024px) {
    .grid-cols-3 {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }

  @media (max-width: 768px) {
    .text-sm {
      font-size: 0.75rem;
    }

    .p-4 {
      padding: 0.5rem;
    }

    .logo {
      width: 50px;
    }

    .grid-cols-3 {
      grid-template-columns: 1fr;
    }
  }

  @media (max-width: 480px) {
    .text-sm {
      font-size: 0.75rem;
    }

    .p-4 {
      padding: 0.25rem;
    }

    .logo {
      width: 40px;
    }

    .grid-cols-3 {
      grid-template-columns: 1fr;
    }
  }
  </style>