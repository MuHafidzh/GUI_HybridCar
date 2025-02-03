import { createRouter, createWebHistory } from 'vue-router';
import HomeView from '../components/home.vue';
import DataMesin from '../components/dataMesin.vue';

const routes = [
  { path: '/', component: HomeView },
  { path: '/data-mesin', component: DataMesin },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;