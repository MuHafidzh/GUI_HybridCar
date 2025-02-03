import { createApp } from "vue";
import { createPinia } from "pinia";
import App from "./App.vue";
import VueKonva from "vue-konva";
import router from "./router";
import "bootstrap/dist/css/bootstrap.min.css";

// Include the css for vue-data-ui
import "vue-data-ui/style.css";

// Import specific components from vue-data-ui
import { VueUiThermometer, VueUiGauge, VueUiWheel, VueUiKpi, VueUiSparkgauge } from "vue-data-ui";

const pinia = createPinia();
const app = createApp(App);

app.use(router);
app.use(VueKonva);
app.use(pinia);

// Register vue-data-ui components globally
app.component("VueUiThermometer", VueUiThermometer);
app.component("VueUiGauge", VueUiGauge);
app.component("VueUiWheel", VueUiWheel);
app.component("VueUiKpi", VueUiKpi);
app.component("VueUiSparkgauge", VueUiSparkgauge);

app.mount("#app");