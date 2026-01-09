// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Book',
  tagline: 'Learn Physical AI, ROS 2, and Robotics',
  favicon: 'img/favicon.svg',

  // Vercel deployment
  url: 'https://physical-ai-book-api.vercel.app',
  baseUrl: '/',
  organizationName: 'faiqahm',
  projectName: 'hackathon_I_book',
  trailingSlash: false,

  // Broken link detection
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Mermaid diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  // i18n configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Book Logo',
          src: 'img/favicon.svg',
          href: '/',
          target: '_self',
        },
        items: [
          {
            to: '/',
            label: 'Home',
            position: 'left',
          },
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/faiqahm/hackathon_I_book',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'custom-authButton',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book.`,
      },
    }),
};

module.exports = config;
