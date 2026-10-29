// The conda-forge commitlint package nests the conventional config inside the CLI.
const cli = `${process.env.CONDA_PREFIX}/lib/node_modules/@commitlint/cli`;

export default {
  extends: [`${cli}/node_modules/@commitlint/config-conventional/lib/index.js`],
};
