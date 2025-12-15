import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'introduction/welcome',
        'introduction/foundations',
        'introduction/learning-outcomes',
        'introduction/sensor-overview',
        'introduction/why-physical-ai-matters',
        'introduction/hardware-requirements',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/introduction',
        'module-1/installation',
        'module-1/core-concepts',
        'module-1/pub-sub',
        'module-1/services-actions',
        'module-1/parameters-packages',
        'module-1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/introduction',
        'module-2/setup',
        'module-2/formats',
        'module-2/physics',
        'module-2/sensors',
        'module-2/unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/overview',
        'module-3/setup',
        'module-3/assets',
        'module-3/synthetic-data',
        'module-3/isaac-ros',
        'module-3/navigation',
        'module-3/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/introduction',
        'module-4/voice-to-text',
        'module-4/cognitive-planning',
        'module-4/language-to-action',
        'module-4/multi-modal',
        'module-4/capstone',
      ],
    },
    {
      type: 'category',
      label: 'Assessments & Projects',
      items: [
        'assessments/overview',
        'assessments/capstone-evaluation',
        'assessments/exercises',
        'assessments/quizzes',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/references',
        'appendices/resources',
        'appendices/troubleshooting',
        'appendices/capstone-project',
        'appendices/ros2-reference',
        'appendices/isaac-resources',
        'appendices/next-steps',
        'appendices/additional-resources',
        'appendices/conclusion',
        'appendices/homework-assignments',
        'appendices/curriculum-evolution',
        'appendices/advanced-glossary',
        'appendices/tasks',
        'appendices/plan',
      ],
    },
  ],
};

export default sidebars;
