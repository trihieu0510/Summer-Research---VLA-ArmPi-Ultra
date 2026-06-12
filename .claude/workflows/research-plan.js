export const meta = {
  name: 'research-plan',
  description: 'Decompose a robotics research question, research each sub-part with web sources, adversarially verify each finding, then synthesize an actionable plan',
  whenToUse: 'Run when you need a rigorously-researched, validated plan or recommendation for the ArmPi Ultra project (next steps, architecture choices, library comparisons).',
  phases: [
    { title: 'Decompose', detail: 'Head agent splits the question into sub-questions' },
    { title: 'Research', detail: 'One researcher per sub-question (web + docs)' },
    { title: 'Verify', detail: 'Adversarial skeptic tries to refute each recommendation' },
    { title: 'Synthesize', detail: 'Head agent merges validated findings into a plan' },
  ],
}

// --- Inputs (passed via the Workflow tool's `args`) ---
const question = (args && args.question) || 'What are the next concrete steps for this robotics project?'
const projectDir = (args && args.projectDir) || ''

// --- Structured output contracts ---
const DECOMP_SCHEMA = {
  type: 'object',
  properties: {
    steps: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          id: { type: 'string' },
          title: { type: 'string' },
          subquestion: { type: 'string' },
          rationale: { type: 'string' },
        },
        required: ['id', 'title', 'subquestion', 'rationale'],
      },
    },
  },
  required: ['steps'],
}

const RESEARCH_SCHEMA = {
  type: 'object',
  properties: {
    summary: { type: 'string' },
    findings: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          claim: { type: 'string' },
          evidence: { type: 'string' },
          source: { type: 'string' },
        },
        required: ['claim', 'evidence', 'source'],
      },
    },
    recommendation: { type: 'string' },
  },
  required: ['summary', 'findings', 'recommendation'],
}

const VERDICT_SCHEMA = {
  type: 'object',
  properties: {
    verdict: { type: 'string', enum: ['supported', 'partly', 'refuted'] },
    confidence: { type: 'number' },
    issues: { type: 'array', items: { type: 'string' } },
    correction: { type: 'string' },
  },
  required: ['verdict', 'confidence', 'issues', 'correction'],
}

// --- Phase 1: the head agent decomposes the question ---
phase('Decompose')
const ctx = projectDir
  ? `Project context: you may Read ${projectDir}\\CLAUDE.md and ${projectDir}\\Documents\\ArmPi_Ultra_VLA_Roadmap.pdf for background.`
  : ''
const decomp = await agent(
  `You are the head/orchestrator of a research team for a Raspberry Pi 5 robotics project (Hiwonder ArmPi Ultra, ROS 2 Humble, modular voice->LLM->vision->IK pipeline).
${ctx}
Break the research question below into 3-5 focused, NON-OVERLAPPING sub-questions that together fully answer it. Each must be concrete and oriented toward an action the user can actually take next.

Research question: ${question}`,
  { label: 'decompose', phase: 'Decompose', schema: DECOMP_SCHEMA }
)

const steps = decomp && decomp.steps ? decomp.steps : []
log(`Decomposed into ${steps.length} sub-questions`)

// --- Phases 2+3: research each sub-question, then adversarially verify it ---
// pipeline = no barrier: a sub-question can be in Verify while another is still in Research.
phase('Research')
const researched = await pipeline(
  steps,
  (step) => agent(
    `Research this sub-question for a Raspberry Pi 5 robotics project (Hiwonder ArmPi Ultra, ROS 2 Humble, NO GPU on the Pi). Use web search for CURRENT, concrete facts: specific libraries, install/run commands, costs, latency numbers, version compatibility. Cite every finding with a URL or doc name. Prefer specific and actionable over general.

Sub-question: ${step.title} — ${step.subquestion}`,
    { label: `research:${step.id}`, phase: 'Research', schema: RESEARCH_SCHEMA }
  ),
  (research, step) => agent(
    `Adversarially verify the recommendation below for a Raspberry Pi 5 / ROS 2 Humble robotics project. Try hard to REFUTE it: look for outdated info, version incompatibilities, hidden costs, or claims that ignore Pi 5 limits (4 cores, 4-8GB RAM, no GPU). Default to skepticism. Return a verdict, concrete issues, and a corrected recommendation if the original is wrong or incomplete.

Sub-question: ${step.title}
Researcher recommendation: ${research ? research.recommendation : '(none)'}
Researcher findings: ${research ? JSON.stringify(research.findings) : '[]'}`,
    { label: `verify:${step.id}`, phase: 'Verify', schema: VERDICT_SCHEMA }
  ).then((v) => ({ step, research, verdict: v }))
)

const valid = researched.filter(Boolean)
log(`Researched + verified ${valid.length} sub-questions`)

// --- Phase 4: head agent synthesizes one actionable plan ---
phase('Synthesize')
const synthesisInput = valid.map((r) => ({
  step: r.step.title,
  summary: r.research ? r.research.summary : null,
  recommendation: r.research ? r.research.recommendation : null,
  verifierVerdict: r.verdict ? r.verdict.verdict : null,
  verifierIssues: r.verdict ? r.verdict.issues : [],
  verifierCorrection: r.verdict ? r.verdict.correction : null,
  sources: r.research ? r.research.findings.map((f) => f.source) : [],
}))

const plan = await agent(
  `You are the head agent. Merge the verified research below into ONE actionable plan for the user's robotics project.
Rules:
- Where a verifier refuted or corrected a finding, trust the CORRECTED version and note the caveat.
- Output a prioritized, ordered list of concrete next steps (commands, libraries, decisions), each with a one-line rationale.
- Make every trade-off explicit (local vs cloud, cost, latency, Pi 5 limits).
- End with a short "Sources" list (dedupe URLs).
Keep it practical and skimmable.

Original question: ${question}

Verified research (JSON):
${JSON.stringify(synthesisInput, null, 2)}`,
  { label: 'synthesize', phase: 'Synthesize' }
)

return plan
